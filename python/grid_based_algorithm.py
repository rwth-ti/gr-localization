import numpy as np
from scipy.signal import resample
import time

def localize(receivers, roi_size):

    t = time.time()
    y = []
    pos_rx = []
    for receiver in receivers:
        y.append(receiver.samples)
        if receiver.selected_position == "manual":
            pos_rx.append(receiver.coordinates)
        else:
            pos_rx.append(receiver.coordinates_gps)
    pos_rx = np.array(pos_rx)

    resolution = 6
    const_c = 300000000
    sample_rate = receivers[0].samp_rate
    channel_model = "free_space"
    num_rx = 3
    num_delayed_samples = receivers[0].samples_to_receive
    num_compressed_samples = num_delayed_samples/10

    (D,mask) = generate_environment_matrices(roi_size, resolution, pos_rx, const_c, sample_rate, channel_model)
    xy = estimate_location_fast(num_rx, pos_rx, roi_size, const_c, resolution, sample_rate, num_compressed_samples, num_delayed_samples, y, D, mask)
    t_used = time.time()-t
    print "Grid based results: ",xy," time: ", t_used
    return {"coordinates": xy,"t_used":t_used}

def estimate_location_fast(num_rx, pos_rx, roi_size, const_c, resolution, sample_rate, num_compressed_samples, num_delayed_samples, y, D, mask):
    M = [0] * num_rx
    for rx_idx in range(0,num_rx):
        M[rx_idx] = generate_measurement_matrix(num_compressed_samples,num_delayed_samples,"gaussian")

    # for normalization
    sigma_1 = np.sqrt(np.sum(np.power(np.abs(y[0]),2)))

    D_masked = [0] * num_rx
    S = [0] * num_rx
    r = [0] * num_rx

    for rx_idx in range(1,num_rx):
        # mask TDOA matrix with environment information to get outdoor area
        D_tmp = D[:,rx_idx]
        masking_idxs = np.ravel_multi_index(np.where(mask==0),(np.ceil((roi_size - float(resolution)/2)/resolution)))
        D_masked[rx_idx] = D_tmp[masking_idxs]
        # take only unique TDOAs at this stage, resample later
        unique_tdoas = np.arange(np.min(D_masked[rx_idx]),np.max(D_masked[rx_idx])+1)
        # create matrices of shifted normalized signals related to reference sensor
        print "size M:", np.array(M[rx_idx]).shape, " size shift: ", np.array(shift_matrix(y[0]/sigma_1,unique_tdoas,2)).shape
        S[rx_idx] = np.dot(M[rx_idx],shift_matrix(y[0]/sigma_1,unique_tdoas,2))
        # compressive sampling of CS sensors
        r[rx_idx] = np.dot(M[rx_idx],y[rx_idx])

    b_joint = np.zeros(len(D_masked[1]))

    phi = [0] * num_rx
    b = [0] * num_rx
    m_idx = [0] * num_rx

    for rx_idx in range(1,num_rx):
        # correlate compressed signal
        phi[rx_idx] = np.dot(S[rx_idx].T,r[rx_idx])

        # resample according to TDOA for each possible position
        b[rx_idx] = phi[rx_idx][(D_masked[rx_idx]-np.min(D_masked[rx_idx])).astype(int)]

        b_joint = b_joint + np.multiply(b[rx_idx],np.conjugate(b[rx_idx]))
    # find Tx position
    m_idx = np.argmax(b_joint)

    # translate to unmasked idx
    m_idx_unmasked = masking_idxs[m_idx]

    # note: flip from matrix subscripts to geometric coordinates
    (pos_x, pos_y) = np.unravel_index(m_idx_unmasked,np.ceil(np.divide(roi_size,float(resolution))))
    tx_pos_hat = (pos_x*resolution+float(resolution)/2,pos_y*resolution+float(resolution)/2)
    return tx_pos_hat

def generate_measurement_matrix(m, n, selection_type):
    if selection_type == "first":
        M = np.eye(m,n)
    elif selection_type == "rand":
        rp = np.random.permutation(n)
        selected_samples = rp[0:m]
        M = np.eye(n)
        M = M[selected_samples]
    elif selection_type == "gaussian":
        M = np.random.randn(m,n)
    return M


def generate_environment_matrices(roi_size, resolution, pos_rx, const_c, sample_rate, channel_model):
    #GENERATE_ENVIRONMENT_MATRICES Generate matrices for TDOA, channel impulse
    #responses and mask of the buildings/environment objects

    # generate impulse responses with ray-tracing
    if channel_model == "piropa":
        print "implement piropa"
        # run piropa to get impulse responses
#        [p_cirs, p_result] = piropa_impulse_responses(cfg)
#        p_mask = environment_mask(cfg)
    else:
        p_mask = np.zeros(np.ceil((roi_size - float(resolution)/2)/resolution))

    p_D = np.zeros((p_mask.size,(len(pos_rx))))
    # calculate TDOA matrices
    for rx_idx in range(1,len(pos_rx)):
        if channel_model == "piropa":
            print "implement piropa"
#            D_tmp = piropa_delay_matrix(cfg,p_cirs{rx_idx}) - piropa_delay_matrix(cfg,p_cirs{1})
        else:
            D_tmp = tdoa_matrix(roi_size, resolution, pos_rx[rx_idx,:], pos_rx[0,:], const_c )

        p_D[:,rx_idx] = np.round(D_tmp.ravel(order="F")*sample_rate)

#    cirs = p_cirs
    D = p_D
    mask = p_mask
#    piropa_result = p_result
    return (D,mask)



def tdoa_matrix(roi_size, resolution, b ,c , const_c):

    x = np.arange(float(resolution)/2,roi_size[1],resolution)
    y = np.arange(float(resolution)/2,roi_size[0],resolution)
    D = np.zeros((len(x),len(y)))
    for x_idx in range(0,len(x)):
        # difference in x direction
        diff_b_x = np.power((x[x_idx]-b[1]),2)
        diff_c_x = np.power((x[x_idx]-c[1]),2)
        # TDOAs
        D[x_idx,:] = (np.sqrt(np.power((y-b[0]),2) + diff_b_x) - np.sqrt(np.power((y-c[0]),2) + diff_c_x))/const_c

    return D

def shift_matrix( s, lags, shift_matrix_oversampling ):

    s_up = resample(s, shift_matrix_oversampling*len(s))
    lags_up = np.round(lags * shift_matrix_oversampling)

    S = np.zeros((len(s),len(lags_up)),dtype=np.complex)

    s_padded = np.concatenate([np.zeros(len(s_up)), s_up, np.zeros(len(s_up))])

    for idx in range(0,len(lags_up)):
        s_shifted = np.roll(s_padded, int(lags_up[idx]))
        s_shifted_down = s_shifted[0:len(s_shifted):shift_matrix_oversampling]
        S[:,idx] = s_shifted_down[len(s):2*len(s)]

    return S


