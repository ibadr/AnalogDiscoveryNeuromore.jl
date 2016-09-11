type RingBuffer
  buffer::Vector{Float64}
  readInd::Int
  writeInd::Int
end

function enque!(buf::RingBuffer,val::Float64)
  len = length(buf.buffer)
  writeInd = buf.writeInd%len + 1
  if writeInd == buf.readInd
    return buf # silently fail
  end
  @inbounds buf.buffer[buf.writeInd] = val
  buf.writeInd = writeInd
  return buf
end

function deque!(buf::RingBuffer)
  if buf.readInd == buf.writeInd
    return NaN # fail
  end
  @inbounds val::Float64 = buf.buffer[buf.readInd]
  len = length(buf.buffer)
  readInd = buf.readInd%len + 1
  buf.readInd = readInd
  return val
end

function used(buf::RingBuffer)
  if buf.writeInd >= buf.readInd
    return buf.writeInd - buf.readInd
  else
    return length(buf.buffer) - (buf.readInd-buf.writeInd)
  end
end
