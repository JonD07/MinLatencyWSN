import subprocess as sp
import time

if __name__ == "__main__":

    sim_process = sp.Popen(["sim_vehicle.py", "-vArduCopter", "-fquad", "-LCSM_SurveyField", "--console", "--map", "--osd"])

    i = 0
    while i < 500:

        con_process = sp.Popen(["python3", "simulation.py", str(i)])
        start_time = time.time()
        return_code = con_process.poll()
        while return_code is None:
            time.sleep(1)
            if(time.time() - start_time > 500.0):
                con_process.terminate()
                sim_process.terminate()
                print("Simulation failed, restarting current iteration.")
                sim_process = sp.Popen(["sim_vehicle.py", "-vArduCopter", "-fquad", "-LCSM_SurveyField", "--console", "--map", "--osd"])
                i -= 1
                break
            return_code = con_process.poll()
        if(return_code is not None and return_code != 0):
            sim_process.terminate()
            con_process.terminate()
            print("Simulation failed with error code {a}, restarting current iteration.".format(a = return_code))
            sim_process = sp.Popen(["sim_vehicle.py", "-vArduCopter", "-fquad", "-LCSM_SurveyField", "--console", "--map", "--osd"])
            i -= 1
        i += 1
