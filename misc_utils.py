
def get_last_packet(sock, bufsize=65536, verbose=False):
    '''Empty out the UDP recv buffer and return only the final packet
    (in case the GUI is slower than the data flow)
    '''
    sock.setblocking(0)
    data = None
    addr = None
    cont = True
    while cont:
        try:
            tmpData, addr = sock.recvfrom(bufsize)
        except Exception as ee:
            #print(ee)
            cont=False
        else:
            if tmpData:
                if data is not None:
                    if verbose:
                        print('throwing away a packet (GUI is too slow)')
                data = tmpData
            else:
                cont=False
    sock.setblocking(1)
    return data, addr



# From StackOverflow:
#   https://stackoverflow.com/questions/1094841

def sizeof_fmt(num, suffix='B'):
    for unit in ['','Ki','Mi','Gi','Ti','Pi','Ei','Zi']:
        if abs(num) < 1024.0:
            return "%3.1f %s%s" % (num, unit, suffix)
        num /= 1024.0
    return "%.1f%s%s" % (num, 'Yi', suffix)


