def main():

    from scripts.commons.Script import Script
    script = Script() #Initialize: load config file, parse arguments, build cpp modules (warns the user about inconsistencies before choosing a test script)

    # Allows using local version of StableBaselines3 (e.g. https://github.com/m-abr/Adaptive-Symmetry-Learning)
    # place the 'stable-baselines3' folder in the parent directory of this project
    import sys
    from os.path import dirname, abspath, join
    sys.path.insert( 0, join( dirname(dirname( abspath(__file__) )), "stable-baselines3") )

    from scripts.commons.UI import UI
    from os.path import isfile, join, realpath, dirname
    from os import listdir, getcwd
    from importlib import import_module

    _cwd = realpath( join(getcwd(), dirname(__file__)))
    gyms_path  = _cwd + "/scripts/gyms/"
    utils_path = _cwd + "/scripts/utils/"
    exclusions = ["__init__.py"]

    utils = sorted([f[:-3] for f in listdir(utils_path) if isfile(join(utils_path, f)) and f.endswith(".py") and f not in exclusions], key=lambda x: (x != "Server", x))
    gyms  = sorted([f[:-3] for f in listdir(gyms_path ) if isfile(join(gyms_path , f)) and f.endswith(".py") and f not in exclusions])

    while True:
        _, col_idx, col = UI.print_table( [utils, gyms], ["Demos & Tests & Utils","Gyms"], cols_per_title=[2,1], numbering=[True]*2, prompt='Choose script (ctrl+c to exit): ' )

        is_gym = False
        if col == 0:
            chosen = ("scripts.utils." , utils[col_idx])
        elif col == 1:
            chosen = ("scripts.gyms." , gyms[col_idx])
            is_gym = True

        cls_name = chosen[1]
        mod = import_module(chosen[0]+chosen[1])

        '''
        An imported script should not automatically execute the main code because:
            - Multiprocessing methods, such as 'forkserver' and 'spawn', will execute the main code in every child
            - The script can only be called once unless it is reloaded
        '''
        if not is_gym:
            ''' 
            Utils receive a script support object with user-defined arguments and useful methods
            Each util must implement an execute() method, which may or may not return
            '''
            from world.commons.Draw import Draw
            from agent.Base_Agent import Base_Agent
            obj = getattr(mod,cls_name)(script)
            try:
                obj.execute() # Util may return normally or through KeyboardInterrupt
            except KeyboardInterrupt:
                print("\nctrl+c pressed, returning...\n")
            Draw.clear_all()            # clear all drawings
            Base_Agent.terminate_all()  # close all server sockets + monitor socket
            script.players = []         # clear list of players created through batch commands
            del obj

        else:
            ''' 
            Gyms must implement a class Train() which is initialized with user-defined arguments and implements:
                train() - method to run the optimization and save a new model
                test(folder_dir, folder_name, model_file) - method to load an existing model and test it
            '''
            from scripts.commons.Train_Base import Train_Base

            print("\nBefore using GYMS, make sure all server parameters are set correctly")
            print("(sync mode should be 'On', real time should be 'Off', cheats should be 'On', ...)")
            print("To change these parameters go to the previous menu, and select Server\n")
            print("Also, GYMS start their own servers, so don't run any server manually")
            
            while True:
                try:
                    idx = UI.print_table([["Train","Test","Retrain"]], numbering=[True], prompt='Choose option (ctrl+c to return): ')[0]
                except KeyboardInterrupt:
                    print()
                    break

                if idx == 0:
                    mod.Train(script).train(dict())
                else:
                    model_info = Train_Base.prompt_user_for_model()
                    if model_info is not None and idx == 1:
                        mod.Train(script).test(model_info)
                    elif model_info is not None:
                        mod.Train(script).train(model_info)


# allow child processes to bypass this file
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nctrl+c pressed, exiting...")
        exit()