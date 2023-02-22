import subprocess as sp
import os
import signal
import time
import defines
import math


if __name__ == "__main__":

    sim_process = sp.Popen(["sim_vehicle.py", "-vArduCopter", "-fquad", "-LCSM_SurveyField", "--console", "--map", "--osd"], preexec_fn=os.setsid)

    time_out = 9600.0
    # num_time_out = 0

    # for n in range(50,110,10):
    #     for j in range(0,5):
            
    #         for m in [0,3]:
    #             repeat = True
    #             while repeat:
    #                 print("Starting "+ defines.MISSION_PATH + "mission_" + str(n) + "_" + str(j) + "_150_2/ mission " + str(m) + " with a " + str(time_out * (2 ** num_time_out)) + " second timer")
    #                 repeat = False
    #                 con_process = sp.Popen(["python3", "simulation.py", str(8), defines.MISSION_PATH + "mission_" + str(n) + "_" + str(j) + "_150_2/", str(m)], preexec_fn=os.setsid)
    #                 start_time = time.time()
    #                 return_code = con_process.poll()
    #                 while return_code is None:
    #                     time.sleep(1)
    #                     if(time.time() - start_time > time_out * (2 ** num_time_out)):
    #                         try:
    #                             con_process.terminate()
    #                             sim_process.terminate()
    #                             os.killpg(os.getpgid(sim_process.pid), signal.SIGTERM)
    #                             os.killpg(os.getpgid(con_process.pid), signal.SIGTERM)
    #                         except:
    #                             print("Termination failed, process already exited")
    #                         print("Simulation failed, restarting current iteration.")
    #                         sim_process = sp.Popen(["sim_vehicle.py", "-vArduCopter", "-fquad", "-LCSM_SurveyField", "--console", "--map", "--osd"], preexec_fn=os.setsid)
    #                         repeat = True
    #                         num_time_out += 1
    #                         break
    #                     elif(math.floor(time.time() - start_time) % 10 == 0):
    #                         print(math.floor(time.time() - start_time))
    #                     return_code = con_process.poll()
    #                 if(return_code is not None and return_code != 0):
    #                     try:
    #                         sim_process.terminate()
    #                         con_process.terminate()
    #                         os.killpg(os.getpgid(sim_process.pid), signal.SIGTERM)
    #                         os.killpg(os.getpgid(con_process.pid), signal.SIGTERM)
    #                     except:
    #                         print("Termination failed, process already exited")
    #                     print("Simulation failed with error code {a}, restarting current iteration.".format(a = return_code))
    #                     sim_process = sp.Popen(["sim_vehicle.py", "-vArduCopter", "-fquad", "-LCSM_SurveyField", "--console", "--map", "--osd"], preexec_fn=os.setsid)
    #                     repeat = True
    
    num_time_out = 0
    for n in range(30,210,10):
        for j in range(0,5):   
            for m in [0,3]:
                repeat = True
                while repeat:
                    print("Starting " + defines.MISSION_PATH + "mission_" + "50_" + str(j) + "_" + str(n) + "_3/ mission " + str(m) + " with a " + str(time_out * (2 ** num_time_out)) + " second timer")
                    repeat = False
                    con_process = sp.Popen(["python3", "simulation.py", str(8), defines.MISSION_PATH + "mission_" + "50_" + str(j) + "_" + str(n) + "_3/", str(m)],  preexec_fn=os.setsid)
                    start_time = time.time()
                    return_code = con_process.poll()
                    while return_code is None:
                        time.sleep(1)
                        if(time.time() - start_time > time_out * (2 ** num_time_out)):
                            try:
                                con_process.terminate()
                                sim_process.terminate()
                                os.killpg(os.getpgid(sim_process.pid), signal.SIGTERM)
                                os.killpg(os.getpgid(con_process.pid), signal.SIGTERM)
                            except:
                                print("simulation process already terminated")
                            print("Simulation failed, restarting current iteration.")
                            sim_process = sp.Popen(["sim_vehicle.py", "-vArduCopter", "-fquad", "-LCSM_SurveyField", "--console", "--map", "--osd"], preexec_fn=os.setsid)
                            repeat = True
                            num_time_out += 1
                            break
                        elif(math.floor(time.time() - start_time) % 10 == 0):
                            print(math.floor(time.time() - start_time))
                        return_code = con_process.poll()
                    if(return_code is not None and return_code != 0):
                        try:
                            sim_process.terminate()
                            con_process.terminate()
                            os.killpg(os.getpgid(sim_process.pid), signal.SIGTERM)
                            os.killpg(os.getpgid(con_process.pid), signal.SIGTERM)
                        except:
                            print("Simulation process alreadyu terminated")
                        print("Simulation failed with error code {a}, restarting current iteration.".format(a = return_code))
                        sim_process = sp.Popen(["sim_vehicle.py", "-vArduCopter", "-fquad", "-LCSM_SurveyField", "--console", "--map", "--osd"],  preexec_fn=os.setsid)
                        repeat = True
                        
                            
