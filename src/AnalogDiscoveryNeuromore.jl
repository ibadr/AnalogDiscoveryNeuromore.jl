module AnalogDiscoveryNeuromore

using AnalogDiscovery

const fs = 512.0
const runForMin = 5
const twoChannels = true
const lowVolt = false

immutable ChannelDoubleBuffer
  chDataBuf1::Vector{Float64}
  chDataBuf2::Vector{Float64}
end

type ChannelState
  writeSecondBuf::Bool
  readSecondBuf::Bool
  waitForWriteFinish::Bool
end

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
  for ch in Int32(0):(twoChannels?Int32(chCount-1):Int32(0))
    analogInChannelEnableSet(hdwf,ch,true)
    analogInChannelFilterSet(hdwf,ch,FILTERAVERAGE)
    analogInChannelRangeSet(hdwf,ch,lowVolt?chVoltMin:chVoltMax)
  end

  sock = UDPSocket(); const localh = ip"127.0.0.1"
  # const fs3_4 = fs*3/4
  # Collect s-sec worth of data
  const s = 60*runForMin
  blks = Int(ceil(s*fs/adbufsize))
  # chData::Vector{Vector{Float64}} = [Vector{Float64}(blks*adbufsize) for i in 1:chCount]
  tBuf = Vector{Float64}(blks*adbufsize)

  # Double-buffering
  ch1dbuf::ChannelDoubleBuffer = ChannelDoubleBuffer(Vector{Float64}(adbufsize),Vector{Float64}(adbufsize))
  ch2dbuf::ChannelDoubleBuffer = ChannelDoubleBuffer(Vector{Float64}(adbufsize),Vector{Float64}(adbufsize))
  chState::ChannelState = ChannelState(false,false,false)
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
  # Discard the first single acquisition for it has transients
  # Start another single acquisition
  analogInConfigure(hdwf,false,true)
  status = analogInStatus(hdwf,true)
  while status != DWFSTATEDONE
    status = analogInStatus(hdwf,true)
  end

  chState.writeSecondBuf = false
  for ch in Int32(0):(twoChannels?Int32(chCount-1):Int32(0))
    chdbuf = (ch==0)?ch1dbuf:ch2dbuf
    writeBuf::Vector{Float64} = chState.writeSecondBuf?chdbuf.chDataBuf2:chdbuf.chDataBuf1
    analogInStatusData!(writeBuf,hdwf,ch)
  end
  # Start another single acquistion for the future
  analogInConfigure(hdwf,false,true)
  status = analogInStatus(hdwf,true)
  chState.writeSecondBuf = true

  # Give 0.1-sec head start
  startT = Libc.time();
  while Libc.time()-startT < 0.1
  end

  startT = Libc.time(); t = -1;
  chState.readSecondBuf = true # to be flipped later as soon as it enters the main loop
  chState.waitForWriteFinish::Bool = false # we already did write one buffer
  @inbounds while t < blks*adbufsize-1
    if status == DWFSTATEDONE
      # @show ("UP",t,chState.waitForWriteFinish,chState.readSecondBuf,chState.writeSecondBuf)
      if chState.waitForWriteFinish || (chState.writeSecondBuf != chState.readSecondBuf) # avoid writing to the same read buffer
        for ch in Int32(0):(twoChannels?Int32(chCount-1):Int32(0))
          # Grab current buffer
          chdbuf = (ch==0)?ch1dbuf:ch2dbuf
          writeBuf::Vector{Float64} = chState.writeSecondBuf?chdbuf.chDataBuf2:chdbuf.chDataBuf1
          analogInStatusData!(writeBuf,hdwf,ch)
        end
        # # Start another single acquistion for the future
        # analogInConfigure(hdwf,false,true)
        chState.writeSecondBuf = !chState.writeSecondBuf
        status = analogInStatus(hdwf,false)
      end
    end

    currT = Libc.time()
    theorT = t/fs+startT
    if (currT>=theorT)
      if !chState.waitForWriteFinish
        t = t+1
      end
      tBuf[t+1] = theorT-currT
      # if t%adbufsize64==0 || t%adbufsize64==1
      #   @show (t,chState.waitForWriteFinish,chState.readSecondBuf,chState.writeSecondBuf)
      # end
      if t%adbufsize64==0 && !chState.waitForWriteFinish
        chState.readSecondBuf = !chState.readSecondBuf
        if chState.writeSecondBuf == chState.readSecondBuf
          chState.waitForWriteFinish = true
        end
      end
      if chState.writeSecondBuf != chState.readSecondBuf
        chState.waitForWriteFinish = false
      end
      idx = t%adbufsize64+1
      for ch in Int32(0):(twoChannels?Int32(chCount-1):Int32(0))
        chdbuf = (ch==0)?ch1dbuf:ch2dbuf
        readBuf::Vector{Float64} = chState.readSecondBuf?chdbuf.chDataBuf2:chdbuf.chDataBuf1
        voltVal = readBuf[idx]
        # chData[ch+1][t+1] = voltVal
        valBuf[1] = Float32(voltVal)
        float32msg!(udpBuf,ch+1,valBuf)
        Base._send(sock,localh,UInt16(4545),udpBuf) # internal _send is faster and we don't care about failures
      end
    end

    if (chState.waitForWriteFinish || (theorT-currT > 0.001))# && ((fs-t%adbufsize64 <= fs3_4) || (t%adbufsize64==0))
      status = analogInStatus(hdwf,false)
      if status == DWFSTATEDONE
        status = analogInStatus(hdwf,true) # read the data (takes >1ms!, eventually accumulates to very large acquisition latency)
        # Start another single acquistion for the future
        analogInConfigure(hdwf,false,true)
        # @show (t%adbufsize64,status)
      end
    end
  end

  deviceClose(hdwf)
  return tBuf
end

end # module
