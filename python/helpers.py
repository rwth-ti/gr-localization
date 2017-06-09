import receiver_interface  

def build_results_strings(receivers):      
    # build receivers strings for log file
    receivers_position = "["
    selected_positions = "["
    receivers_gps = "["
    receivers_antenna = "["
    receivers_gain = "["
    receivers_offset = "["
    i = 1
    for receiver in receivers.values():
        if i == 1:
            #TODO: general solution!!!
            if receiver.selected_position == "manual":
                receivers_position = receivers_position + str(receiver.coordinates)
            elif receiver.selected_position == "selfloc":
                receivers_position = receivers_position + str(receiver.coordinates_selfloc)
            else:
                receivers_position = receivers_position + str(receiver.coordinates_gps)
            selected_positions = selected_positions + "'" + receiver.selected_position + "'"
            receivers_gps = receivers_gps + "'" + receiver.gps + "'"
            receivers_antenna = receivers_antenna + "'" + receiver.antenna + "'"
            receivers_gain = receivers_gain + str(receiver.gain)
            receivers_offset = receivers_offset + str(receiver.offset)
        else:
            if receiver.selected_position == "manual":
                receivers_position = receivers_position + "," + str(receiver.coordinates)
            elif receiver.selected_position == "selfloc":
                receivers_position = receivers_position + "," + str(receiver.coordinates_selfloc)
            else:
                receivers_position = receivers_position + "," + str(receiver.coordinates_gps)
            selected_positions = selected_positions + "," + "'" + receiver.selected_position + "'"
            receivers_gps = receivers_gps + "," + "'" + receiver.gps + "'"
            receivers_antenna = receivers_antenna + "," + "'" + receiver.antenna + "'"
            receivers_gain = receivers_gain + "," + str(receiver.gain)
            receivers_offset = receivers_offset + "," + str(receiver.offset)
        i = i + 1
    receivers_position = receivers_position + "]"
    selected_positions = selected_positions + "]"
    receivers_gps = receivers_gps + "]"
    receivers_antenna = receivers_antenna + "]"
    receivers_gain = receivers_gain + "]"
    receivers_offset = receivers_offset + "]"
    return receivers_position, selected_positions, receivers_gps, receivers_antenna, receivers_gain, receivers_offset
    