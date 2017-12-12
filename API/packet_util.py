def getHeader(functionid, componentid):
    return (functionid<<2) + componentid

def getPacket(fid, cid, data):
    packet = list()
    packet.append(getHeader(fid,cid))
    packet.append(len(data))
    packet.extend(data)
    return packet

def isEmpty(response):
    numbytes = len(response)
    for i in range(0,numbytes):
        if response[i] is None:
            return True
    return False