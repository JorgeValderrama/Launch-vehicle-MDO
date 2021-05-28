# -*- coding: utf-8 -*-
"""
Created on Wed Mar 01 15:08:27 2017

This class calculates the dry mass of the second stage as in FELIN.
Dry mass of second stage is only function of the mass of propellants.

@author: lbrevaul
@modified: jorge
"""

from __future__ import print_function
# import numpy as np
from openmdao.api import ExplicitComponent

class  Dry_Mass_stage_2_Comp(ExplicitComponent):
    
    def setup(self):
        
        self.add_input('Prop_mass_stage_2',
                       val =0.0,
                       units = 'kg',
                       desc = 'mass of propellants stage 2')
        
        self.add_output('Dry_mass_stage_2',
                        val =0.0,
                        units = 'kg',
                        desc = ' dry mass of stage 2')
        
        self.declare_partials( of = 'Dry_mass_stage_2' , wrt = 'Prop_mass_stage_2')
        
    def compute(self, inputs , outputs):
        
        Prop_mass_stage_2 = inputs ['Prop_mass_stage_2']
        
        #Regression to estimate the dry mass (without the propellant) of the second stage as a function of the propellant mass
        # outputs['Dry_mass_stage_2'] = (80.0*(Prop_mass_stage_2/1e3)**(-0.5)) / 100 * Prop_mass_stage_2 ## Transcost MODEL
        outputs['Dry_mass_stage_2'] = 0.8 * 1000**0.5 * Prop_mass_stage_2**0.5
        
    def compute_partials(self, inputs, jacobian):
        
        Prop_mass_stage_2 = inputs ['Prop_mass_stage_2']
        
        jacobian['Dry_mass_stage_2', 'Prop_mass_stage_2'] = 0.8 * 1000**0.5 *0.5 * Prop_mass_stage_2**(-0.5)
        
        