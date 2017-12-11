def getHeader(functionid, componentid)
    return (componentid<<2) + componentid

def getPacket(fid, cid, data)
    packet = list()
    packet.append(getHeader(fid,cid))
    packet.append(len(data))
    packet.extend(data)
    return packet