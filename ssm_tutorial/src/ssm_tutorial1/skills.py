#!/usr/bin/env python

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : loop_skill.py
# Authors : Ludovic DELVAL
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Ludovic DELVAL <ludovic.delval.external@airbus.com>
#
#
################################################################################

from ssm_core_v2 import ssm_state

import rospy

        
class Input(ssm_state.ssmState):
    ##Describe a loop which check if i_for < cond_for
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["Test","Out","Retry"], 
                                    io_keys=["input"])
        
        
    def execution(self, ud):
        print("Prime Tester ('q' to print all prime number found) :")
        input = raw_input()
        if(input == 'q'):
            return 'Out'
        else:
            try:
                ud.input = int(input)
                return "Test"
            except:
                print("[Error] Couldn't convert "+str(input) +" into an int !")
                return "Retry"
class isPrime(ssm_state.ssmState):
    ##Describe a loop which check if i_for < cond_for
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["Return"], 
                                    io_keys=["input","prime_list"])
        
    def is_prime(self, a):
        return all(a % i for i in xrange(2, a))
    
    def execution(self, ud):
        ## Test if it's prime_list is not initialised
        if ud.prime_list is None:
            ud.prime_list = []
        
        if(self.is_prime(ud.input)):
            print(str(ud.input) + " is a prime number.")
            ud.prime_list.append(ud.input)
        else:
            print(str(ud.input) + " is not a prime number.")
            
        return "Return"
            
            
class Primes(ssm_state.ssmState):
    ##Describe a loop which check if i_for < cond_for
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["Reset","Off", "Continue"], 
                                    io_keys=["prime_list"])
        
    
    def execution(self, ud):
        ## Test if it's prime_list is not initialised
        if ud.prime_list is None:
            ud.prime_list = []
            print("Prime list is empty.")
        else:
            print("Prime list :")
            for i_prime in ud.prime_list:
                print(str(i_prime))
        while(1):
            print("'c' to continue")
            print("'r' to reset")
            print("'q' to quit\n")
            input = raw_input("input : ")
            if(input == 'c'):
                return "Continue"
            elif(input == 'r'):
                ud.prime_list = []
                return 'Reset'
            elif(input == 'q'):
                return 'Off'
            else:
                print("Input not recognize")
    
        