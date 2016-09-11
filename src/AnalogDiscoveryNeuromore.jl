module AnalogDiscoveryNeuromore

using AnalogDiscovery

include("RingBuffer.jl")

const fs = 512.0
const runForMin = 0.5
const twoChannels = true
const lowVolt = false

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
  analogInAcquisitionModeSet(hdwf,ACQMODERECORD)
  analogInRecordLengthSet(hdwf,0.0) # run indefinitely

  chVoltMin,chVoltMax,chSteps = analogInChannelRangeInfo(hdwf)
  chCount = analogInChannelCount(hdwf)
  for ch in Int32(0):(twoChannels?Int32(chCount-1):Int32(0))
    analogInChannelEnableSet(hdwf,ch,true)
    analogInChannelFilterSet(hdwf,ch,FILTERAVERAGE)
    analogInChannelRangeSet(hdwf,ch,lowVolt?chVoltMin:chVoltMax)
  end

  sock = UDPSocket(); const localh = ip"127.0.0.1"
  const fs_Int32 = round(Int32,fs)
  # Collect s-sec worth of data
  const s = 60*runForMin
  blks = Int(ceil(s*fs/adbufsize))
  # chData::Vector{Vector{Float64}} = [Vector{Float64}(blks*adbufsize) for i in 1:chCount]
  tBuf = Vector{Float64}(blks*adbufsize)

  # RingBuffers
  ch1buf = RingBuffer(Vector{Float64}(adbufsize),1,1)
  ch2buf = RingBuffer(Vector{Float64}(adbufsize),1,1)
  # Single-element buffer to reinterpret Float32 value later using unsafe_load
  valBuf::Vector{Float32} = Vector{Float32}(1)
  # Buffer for OSC packets sent over UDP
  udpBuf = Array{UInt8,1}(16)

  # sleep(2.0)
  analogInConfigure(hdwf,false,true) # Start acquistion

  # Prefill
  status = analogInStatus(hdwf,true)
  while status != DWFSTATETRIGGEREDRUNNING
    status = analogInStatus(hdwf,true)
  end
  cSamples = 0
  chDataBuf = Vector{Float64}(adbufsize)
  cAvailable,cLost,cCorrupted = analogInStatusRecord(hdwf)
  cSamples += cLost
  @inbounds for ch in Int32(0):Int32(chCount-1)
    analogInStatusData!(chDataBuf,hdwf,ch,Int32(cAvailable))
    # chData[ch+1][cSamples+(1:cAvailable)] = chDataBuf[1:cAvailable]
    for n in 1:cAvailable
      if ch==0
        ch1buf = enque(ch1buf,chDataBuf[n])
      else
        ch2buf = enque(ch2buf,chDataBuf[n])
      end
    end
  end
  cSamples += cAvailable

  nSamples = blks*adbufsize
  startT = Libc.time(); t = -1
  while t < nSamples-1
    if used(ch1buf) == 0
      status = analogInStatus(hdwf,true)
      if (cSamples == 0) && ((status == DWFSTATECONFIG) || (status == DWFSTATEPREFILL) || (status == DWFSTATEARMED))
        # Acquisition not yet started
        continue
      end
      cAvailable,cLost,cCorrupted = analogInStatusRecord(hdwf)
      # @show (cSamples,cAvailable,cLost,cCorrupted)
      cSamples += cLost
      if cAvailable==0
        continue
      end
      if cSamples+cAvailable > nSamples
        cAvailable = nSamples-cSamples
      end
      # get samples
      @inbounds for ch in Int32(0):Int32(chCount-1)
        analogInStatusData!(chDataBuf,hdwf,ch,Int32(cAvailable))
        # chData[ch+1][cSamples+(1:cAvailable)] = chDataBuf[1:cAvailable]
        for n in 1:cAvailable
          if ch==0
            ch1buf = enque(ch1buf,chDataBuf[n])
          else
            ch2buf = enque(ch2buf,chDataBuf[n])
          end
        end
      end
      cSamples += cAvailable
    end

    # Now send UDP samples
    currT = Libc.time()
    theorT = t/fs+startT
    if (currT>=theorT)
      if used(ch1buf) > 0
        # @show used(ch1buf)
        t += 1
        tBuf[t+1] = theorT-currT
        for ch in Int32(0):(twoChannels?Int32(chCount-1):Int32(0))
          if ch==0
            (voltVal,ch1buf) = deque(ch1buf)
          else
            (voltVal,ch2buf) = deque(ch2buf)
          end
          valBuf[1] = Float32(voltVal)
          float32msg!(udpBuf,ch+1,valBuf)
          Base._send(sock,localh,UInt16(4545),udpBuf) # internal _send is faster and we don't care about failures
        end
      end
    end
  end

  deviceClose(hdwf)
  return tBuf
end

end # module
