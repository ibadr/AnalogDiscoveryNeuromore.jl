module AnalogDiscoveryNeuromore

using AnalogDiscovery

const fs=512.0

function float32msg!(buffer::Vector{UInt8},ch::Int64,val::Vector{Float32})
  buffer[1:5]=(ch==1) ? "/in/0".data : "/in/1".data
  @inbounds for i in 6:8
    buffer[i]=UInt8(0)
  end
  buffer[9:10]=",f".data
  @inbounds for i in 11:12
    buffer[i]=UInt8(0)
  end
  @inbounds for i in 1:4
    buffer[12+i]=unsafe_load(Ptr{UInt8}(pointer(val)),4-i+1)
  end
  #@show buffer
  return buffer
end

function go()
  cDev = enumDevices()
  @show ("Available",cDev)
  myDev = Int32(0)
  if cDev > 0
    if enumDeviceIsOpened(myDev)
      error("Error opening device, already in use.")
    end
    hdwf = deviceOpen(myDev)
  else
    error("No devices available.")
  end

  bmin,bmax = analogInBufferSizeInfo(hdwf)
  adbufsize = Int32(min(fs,bmax))
  adbufsize64 = Int64(adbufsize)
  analogInFrequencySet(hdwf,fs)
  analogInBufferSizeSet(hdwf,adbufsize)
  analogInAcquisitionModeSet(hdwf,ACQMODESINGLE)

  chVoltMin,chVoltMax,chSteps = analogInChannelRangeInfo(hdwf)
  chCount = analogInChannelCount(hdwf)
  for ch in Int32(0):Int32(chCount-1)
    analogInChannelEnableSet(hdwf,ch,true)
    analogInChannelFilterSet(hdwf,ch,FILTERAVERAGE)
    analogInChannelRangeSet(hdwf,ch,chVoltMin)
  end

  sock = UDPSocket(); const localh = ip"127.0.0.1"
  # Collect s-sec worth of data
  const s = 20
  blks = Int(ceil(s*fs/adbufsize))
  # chData::Vector{Vector{Float64}} = [Vector{Float64}(blks*adbufsize) for i in 1:chCount]
  tBuf = Vector{Float64}(blks*adbufsize)

  # Double-buffering
  chDataBuf1::Vector{Float64} = Vector{Float64}(adbufsize)
  chDataBuf2::Vector{Float64} = Vector{Float64}(adbufsize)
  # Single-element buffer to reinterpret Float32 value later using unsafe_load
  valBuf::Vector{Float32} = Vector{Float32}(1)
  # Buffer for OSC packets sent over UDP
  udpBuf = Array{UInt8,1}(16)

  # Prefill
  analogInConfigure(hdwf,false,true) # Start a single acquistion
  status = analogInStatus(hdwf,true)
  while status != DWFSTATEDONE
    status = analogInStatus(hdwf,true)
  end
  writeBufSel = 0; writeBuf::Vector{Float64} = chDataBuf1
  ch=1; analogInStatusData!(writeBuf,hdwf,Int32(ch-1))
  # Start another single acquistion for the future
  analogInConfigure(hdwf,false,true)
  status = analogInStatus(hdwf,true)
  writeBufSel = 1

  readBufSel = 0; readBuf::Vector{Float64} = chDataBuf1

  startT = Libc.time(); t = -1;
  @inbounds while t < blks*adbufsize-1
    if status == DWFSTATEDONE
      # Grab current buffer
      writeBuf = (writeBufSel == 0) ? chDataBuf1 : chDataBuf2
      analogInStatusData!(writeBuf,hdwf,Int32(ch-1))
      # Start another single acquistion for the future
      analogInConfigure(hdwf,false,true)
      writeBufSel = (writeBufSel+1)%2
      status = analogInStatus(hdwf,true)
    end

    currT = Libc.time()
    theorT = t/fs+startT
    if (currT>=theorT)
      t = t+1
      tBuf[t+1] = theorT-currT
      if t%adbufsize64==0
        readBufSel = (readBufSel+1)%2
        readBuf = (readBufSel==0) ? chDataBuf1 : chDataBuf2
      end
      idx = t%adbufsize64+1
      voltVal = readBuf[idx]
      # chData[ch][t+1] = voltVal
      valBuf[1] = Float32(voltVal)
      float32msg!(udpBuf,ch,valBuf)
      Base._send(sock,localh,UInt16(4545),udpBuf) # internal _send is faster and we don't care about failures
    elseif (theorT-currT > 0.001)
      status = analogInStatus(hdwf,true)
    end
  end

  deviceClose(hdwf)
  return tBuf
end

tBuf = go()

end # module
