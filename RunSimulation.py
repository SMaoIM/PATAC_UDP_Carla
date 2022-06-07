import argparse
import importlib
import sys



def arg_parse():
    argparser = argparse.ArgumentParser(
        description='PATAC ADAS Simulation Client')
    argparser.add_argument(
        '-a', '--algorithm',
        metavar='A',
        default='ACC_LCC_50ms',
        help='choose algoritm to test')
    

    opt = argparser.parse_args()
    return opt


def main():
    opt = arg_parse()

    try:
        testing_scenario = importlib.import_module("TestScenario.%s" % opt.algorithm)
    except ModuleNotFoundError:
        sys.exit("ERROR: %s.py is not exist in TestScenario/ " % opt.algorithm)

    
    RUN_TEST = getattr(testing_scenario, 'RUN_TEST')
    # run scenario testing
    RUN_TEST()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')