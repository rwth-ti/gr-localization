import receiver_interface  

def build_results_strings(receivers):      
    # build receivers strings for log file
    receivers_position = "["
    selected_positions = "["
    receivers_gps = "["
    receivers_antenna = "["
    receivers_gain = "["
    i = 1
    for receiver in receivers.values():
        if i == 1:
            #TODO: general solution!!!
            if receiver.selected_position in ["manual", "selfloc"]:
                receivers_position = receivers_position + str(receiver.coordinates)
            else:
                receivers_position = receivers_position + str(receiver.coordinates_gps)
            selected_positions = selected_positions + "'" + receiver.selected_position + "'"
            receivers_gps = receivers_gps + "'" + receiver.gps + "'"
            receivers_antenna = receivers_antenna + "'" + receiver.antenna + "'"
            receivers_gain = receivers_gain + str(receiver.gain)
        else:
            if receiver.selected_position in ["manual", "selfloc"]:
                receivers_position = receivers_position + "," + str(receiver.coordinates)
            else:
                receivers_position = receivers_position + "," + str(receiver.coordinates_gps)
            selected_positions = selected_positions + "," + "'" + receiver.selected_position + "'"
            receivers_gps = receivers_gps + "," + "'" + receiver.gps + "'"
            receivers_antenna = receivers_antenna + "," + "'" + receiver.antenna + "'"
            receivers_gain = receivers_gain + "," + str(receiver.gain)
        i = i + 1
    receivers_position = receivers_position + "]"
    selected_positions = selected_positions + "]"
    receivers_gps = receivers_gps + "]"
    receivers_antenna = receivers_antenna + "]"
    receivers_gain = receivers_gain + "]"
    return receivers_position, selected_positions, receivers_gps, receivers_antenna, receivers_gain