sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
function [returnCode] = send_path_4_drawing(path, sleep_time)
    %the bigger the sleep time the more accurate the points are placed but you have to be very patient :D
    for i=path
        packedData=sim.simxPackFloats(i.flatten());
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData)
        returnCode=sim.simxWriteStringStream(clientID, "path_coord", raw_bytes, sim.simx_opmode_oneshot)
        time.sleep(sleep_time)
        
    end
end