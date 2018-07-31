import numpy as np
import rpi_abb_irc5
import time
import collections
import scipy.io
import matplotlib.pyplot as plt
import sys
from scipy.optimize import minimize_scalar
import winsound
	
def first_half(x_inc6):
    # stop the active RAPID program
    rapid.stop()
    # reset the pointer to main
    rapid.resetpp()
    time.sleep(2) 
    print 'restart'
    # start the RAPID program
    rapid.start()
    
    tag = True
		
    while tag:
        res, state = egm.receive_from_robot(.1)
        if res:
            # guarantee the robot has moved to initial configuration
            if np.fabs(sum(np.rad2deg(state.joint_angles)) - sum(state_init)) < 1e-3:
                tag = False
    
    time.sleep(0.5)	
    
    egm_joint = np.zeros((6, n))

    t1 = time.time()
    t_pre = time.time()-t1
    count = 0
    while count < n:
    
        res, state = egm.receive_from_robot(0.1)
        
        if not res:
            continue
        
        b = x_inc6[:, count]
        
        # convert into radians
        egm.send_to_robot(np.deg2rad(b))
        egm_joint[:, count] = np.rad2deg(state.joint_angles)
        
        count = count + 1
        
    error = egm_joint - desired
    # flip the error
    err_flip = np.fliplr(error)
    print np.linalg.norm(error, 'fro')
	
    return egm_joint, err_flip, np.linalg.norm(error, 'fro')

# object function (Frobenius error) for 1d learning rate search
def object_function(x):
    
    rapid.stop()
    rapid.resetpp()
    time.sleep(2.0) 
    print 'restart'
    rapid.start()
   
    tag = True
  
    while tag:
        res, state = egm.receive_from_robot(.1)
        if res:
            if np.fabs(sum(np.rad2deg(state.joint_angles)) - sum(state_init)) < 1e-3:
                tag = False
	
    time.sleep(0.5)	
    
    print '--------start EGM-----------'
    
    egm_joint = np.zeros((6, n))
    
    x_in = x_inc6-x*errflip2
	
    t1 = time.time()
    t_pre = time.time()-t1
    count = 0
    while count < n:
    
        res, state = egm.receive_from_robot(0.1)
        
        if not res:
            continue
        
        b = x_in[:, count]
        
        # convert into radians
        egm.send_to_robot(np.deg2rad(b))
        egm_joint[:, count] = np.rad2deg(state.joint_angles)
        
        count = count + 1
    
    error = egm_joint - desired
    time.sleep(3)
	
    return np.linalg.norm(error, 'fro')
    
def second_half(x, out):
    rapid.stop()
    rapid.resetpp()
    time.sleep(2) 
    print 'restart'
    rapid.start()
		
    tag = True
	
    while tag:
        res, state = egm.receive_from_robot(.1)
        if res:
            if np.fabs(sum(np.rad2deg(state.joint_angles)) - sum(state_init)) < 1e-3:
                tag = False
         
    time.sleep(0.5)
    	
    egm_joint = np.zeros((6, n))
   
    x_inc6 = x
    
    t1 = time.time()
    t_pre = time.time()-t1
    count = 0
    
    while count < n:
        res, state = egm.receive_from_robot(0.1)
        
        if not res:
            continue
        
        b = x_inc6[:, count]
        
        egm.send_to_robot(np.deg2rad(b))
        egm_joint[:, count] = np.rad2deg(state.joint_angles)
        
        count = count + 1
    err = egm_joint-out
    err_flip2 = np.fliplr(err)
    
    return err_flip2

	
########################
######change here#######	
mat_inc6 = scipy.io.loadmat('training_data_78.mat')
x_inc6 = mat_inc6['qd']
print x_inc6[:, 0]
time.sleep(3)

# initial state
state_init = x_inc6[:, 0] 

# create object for connection with EGM  
egm=rpi_abb_irc5.EGM()
	
n=x_inc6.shape[1]

# desired trajectory
desired = mat_inc6['qd']
        
if (len(sys.argv) >= 2):
    rapid=rpi_abb_irc5.RAPID(sys.argv[1])
else:
    rapid=rpi_abb_irc5.RAPID()

fro_err_old = 0
for x in range(15):
   
    out, err_flip1, fro_err = first_half(x_inc6)
    
    # check if the stopping condition satisfied
    if np.fabs(fro_err-fro_err_old) < 1:
        csv_dat=np.hstack((desired.T, x_inc6.T))
		
        ########################
        ######change here#######
        np.savetxt('training_data_78_final.csv', csv_dat, fmt='%6.5f', delimiter=',')#, header='desired joint, optimal input')
        frequency = 2500  # Set Frequency To 2500 Hertz
        duration = 1000  # Set Duration To 1000 ms == 1 second
        winsound.Beep(frequency, duration)
        break;
	
    time.sleep(5)   
    
    x = x_inc6+err_flip1	 
    errflip2 = second_half(x, out)
    
    time.sleep(5)  
	
    print '----start searching optimal learning rate.----'
    res = minimize_scalar(object_function, bounds=(0.0, 1.0), method='bounded', options={'maxiter': 5})
    print '----the optimal learning rate is:----' 
    print res.x
	
    # update input
    x_inc6 = x_inc6 - res.x*errflip2
    
    fro_err_old = fro_err