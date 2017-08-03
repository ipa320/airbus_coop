################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : parameters.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

class Parameters():
    
    def __init__(self):
        self._params = {}
        
    def putParam(self, name, values):
        
        if not self._params.has_key(name):
            self._params.update({name:values})
        else:
            self._params[name] = values
        
    def getParam(self, name, default=None):
        
        if self._params.has_key(name):
            return self._params[name]
        elif default is not None:
            return default
        
        return None
    
    def __str__(self):
        
        params_str = "Parameters :\n"
        
        for k,v in self._params.items():
            params_str+= "  - %s : %s\n"%(k,str(v))
            
        return params_str
    
    