import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl
from fcl_parser import FCLParser
import CarSim2

# First we set up the variables in the usual way:

# New Antecedent/Consequent objects hold universe variables and membership
# functions
PI = 3.14159
miniPI = 10*PI/360

#Output Data
phi_degree = 9*PI/180
phi = ctrl.Consequent(np.arange(-PI/4, PI/4, miniPI), 'phi')        # change of angles of the driver wheels方向盤轉動角度
velocity = ctrl.Consequent(np.arange(-5, 5, 0.1), 'velocity')   # velocity速度
#phaseOut = ctrl.Consequent(np.arange(1, 11,1), 'phaseOut')
phaseOut = ctrl.Consequent(np.arange(1, 12,1), 'phaseOut')

##  Input Data
theta = ctrl.Antecedent(np.arange(-PI, PI, miniPI), 'theta')  # angles of the car relative to the horizontal line
DistXr2Tip1 = ctrl.Antecedent(np.arange(-512, 512, 1), 'DistXr2Tip1')  # Distance to Tip1 from rear wheel axis
DistXr2Tip2 = ctrl.Antecedent(np.arange(-512, 512, 1), 'DistXr2Tip2')  # Distance to Tip2 rear wheel axis
DistXf2Tip1 = ctrl.Antecedent(np.arange(-512, 512, 1), 'DistXf2Tip1')  # Distance to Tip1 from front wheel axis
#phase = ctrl.Antecedent(np.arange(1, 11,1), 'phase')
phase = ctrl.Antecedent(np.arange(1, 12,1), 'phase')  
currentPhi = ctrl.Antecedent(np.arange(-PI, PI, miniPI), 'currentPhi')  

# Auto-membership function population is possible with .automf(3, 5, or 7)
'''
phi.automf(5)
velocity.automf(5)
theta.automf(3)
DistXr2Tip1.automf(3)
DistXr2Tip2.automf(3)
'''
# Custom membership functions can be built interactively with a familiar,
# Pythonic API
# unit: 9 degree, about 9*PI/180

currentPhi['largeNegative'] = fuzz.trimf(currentPhi.universe, [-PI, -3*phi_degree*5, -1.5*phi_degree*5])
#currentPhi['smallNegative'] = fuzz.trimf(currentPhi.universe, [-2*phi_degree*5, -phi_degree*5, -0.3*phi_degree*5])
currentPhi['smallNegative'] = fuzz.trimf(currentPhi.universe, [-2*phi_degree*5, -phi_degree*5, -0.02])
#currentPhi['zero'] = fuzz.trimf(currentPhi.universe, [-0.4*phi_degree*5, 0, 0.4*phi_degree*5])
currentPhi['zero'] = fuzz.trimf(currentPhi.universe, [-0.03, 0, 0.03])
#currentPhi['smallPositive'] = fuzz.trimf(currentPhi.universe, [0.3*phi_degree*5, phi_degree*5, 2*phi_degree*5])
currentPhi['smallPositive'] = fuzz.trimf(currentPhi.universe, [0.02, phi_degree*5, 2*phi_degree*5]) 
currentPhi['largePositive'] = fuzz.trimf(currentPhi.universe, [1.5*phi_degree*5, 3*phi_degree*5, PI])


phi['largeNegative'] = fuzz.trimf(phi.universe, [-PI/4, -3*phi_degree, -1.5*phi_degree])
phi['smallNegative'] = fuzz.trimf(phi.universe, [-2*phi_degree, -phi_degree, -0.3*phi_degree])
phi['zero'] = fuzz.trimf(phi.universe, [-0.4*phi_degree, 0, 0.4*phi_degree])
phi['smallPositive'] = fuzz.trimf(phi.universe, [0.3*phi_degree, phi_degree, 2*phi_degree]) 
phi['largePositive'] = fuzz.trimf(phi.universe, [1.5*phi_degree, 3*phi_degree, PI/4])

velocity['backFaster'] = fuzz.trimf(velocity.universe, [-5, -4, -3])
velocity['backSlower'] = fuzz.trimf(velocity.universe, [-5, -2.5, 0])
velocity['zero'] = fuzz.trimf(velocity.universe, [-1, 0.0, 1.0])
velocity['forwardSlower'] = fuzz.trimf(velocity.universe, [0, 2.5, 5])
velocity['forwardFaster'] = fuzz.trimf(velocity.universe, [3, 4, 5])

theta['negative45'] = fuzz.trimf(theta.universe, [-50*PI/180, -PI/4, -43*PI/180])
#theta['zero'] = fuzz.trimf(theta.universe, [-1, 0.0, 1.0])
theta['zero'] = fuzz.trimf(theta.universe, [-1*PI/180, 0.0, 1*PI/180])
theta['positive45'] = fuzz.trimf(theta.universe, [43*PI/180, PI/4, 50*PI/180])


DistXr2Tip1['negativeFar'] = fuzz.trimf(DistXr2Tip1.universe, [-512, -30, -15.0])
DistXr2Tip1['negativeNear'] = fuzz.trimf(DistXr2Tip1.universe, [-20, -10, -4.0])
DistXr2Tip1['near'] = fuzz.trimf(DistXr2Tip1.universe, [-5.0, 0.0, 5.0])
DistXr2Tip1['positiveNear'] = fuzz.trimf(DistXr2Tip1.universe, [4.0, 10.0, 20])
DistXr2Tip1['positiveFar'] = fuzz.trimf(DistXr2Tip1.universe, [15.0, 30.0, 512])
DistXr2Tip1['positiveFar'].view()
DistXr2Tip2['negativeFar'] = fuzz.trimf(DistXr2Tip2.universe, [-512, -120, -95.0])
DistXr2Tip2['negativeNear'] = fuzz.trimf(DistXr2Tip2.universe, [-100, -90, -85.0])

#DistXr2Tip2['near'] = fuzz.trimf(DistXr2Tip2.universe, [-90.0, 0.0, 90.0])
DistXr2Tip2['near'] = fuzz.trimf(DistXr2Tip2.universe, [-40.0, 0.0, 120.0])

DistXr2Tip2['positiveNear'] = fuzz.trimf(DistXr2Tip2.universe, [85.0, 90.0, 100])
DistXr2Tip2['positiveFar'] = fuzz.trimf(DistXr2Tip2.universe, [95.0, 120.0, 512])

DistXf2Tip1['negativeFar'] = fuzz.trimf(DistXf2Tip1.universe, [-512, -50, -35.0])


DistXf2Tip1['negativeNear'] = fuzz.trimf(DistXf2Tip1.universe, [-40, -30, -24.0])

DistXf2Tip1['near'] = fuzz.trimf(DistXf2Tip1.universe, [-25.0, 0.0, 25.0])


DistXf2Tip1['positiveNear'] = fuzz.trimf(DistXf2Tip1.universe, [24.0, 30.0, 40])

DistXf2Tip1['positiveFar'] = fuzz.trimf(DistXf2Tip1.universe, [35.0, 50.0, 512])


phase['one'] = fuzz.trimf(phase.universe, [1, 1, 1])
phase['two'] = fuzz.trimf(phase.universe, [2, 2, 2])
phase['three'] = fuzz.trimf(phase.universe, [3, 3, 3])
phase['four'] = fuzz.trimf(phase.universe, [4, 4, 4])
phase['five'] = fuzz.trimf(phase.universe, [5, 5, 5])
phase['six'] = fuzz.trimf(phase.universe, [6, 6, 6])
phase['seven'] = fuzz.trimf(phase.universe, [7, 7, 7])
phase['eight'] = fuzz.trimf(phase.universe, [8, 8, 8])
phase['nine'] = fuzz.trimf(phase.universe, [9, 9, 9])
phase['ten'] = fuzz.trimf(phase.universe, [10, 10, 10])

phase['eleven'] = fuzz.trimf(phase.universe, [11, 11, 11])

phaseOut['one'] = fuzz.trimf(phaseOut.universe, [1, 1, 1])
phaseOut['two'] = fuzz.trimf(phaseOut.universe, [2, 2, 2])
phaseOut['three'] = fuzz.trimf(phaseOut.universe, [3, 3, 3])
phaseOut['four'] = fuzz.trimf(phaseOut.universe, [4, 4, 4])
phaseOut['five'] = fuzz.trimf(phaseOut.universe, [5, 5, 5])
phaseOut['six'] = fuzz.trimf(phaseOut.universe, [6, 6, 6])
phaseOut['seven'] = fuzz.trimf(phaseOut.universe, [7, 7, 7])
phaseOut['eight'] = fuzz.trimf(phaseOut.universe, [8, 8, 8])
phaseOut['nine'] = fuzz.trimf(phaseOut.universe, [9, 9, 9])
phaseOut['ten'] = fuzz.trimf(phaseOut.universe, [10, 10, 10])

phaseOut['eleven'] = fuzz.trimf(phaseOut.universe, [11, 11, 11])

#print("XXXX:", fuzz.interp_membership(np.arange(-PI, PI, miniPI), theta['zero'], 0.0) )

"""
To help understand what the membership looks like, use the ``view`` methods.
"""

# You can see how these look with .view()
"""
phi.view()
velocity.view()
theta.view()
DistXr2Tip1.view()
DistXr2Tip2.view()
"""


"""
Fuzzy rules
-----------

Now, to make these triangles useful, we define the *fuzzy relationship*
between input and output variables. For the purposes of our example, consider
three simple rules:

1. If the food is poor OR the service is poor, then the tip will be low
2. If the service is average, then the tip will be medium
3. If the food is good OR the service is good, then the tip will be high.

Most people would agree on these rules, but the rules are fuzzy. Mapping the
imprecise rules into a defined, actionable tip is a challenge. This is the
kind of task at which fuzzy logic excels.
"""
# Now use FCL to define three rules:
p = FCLParser()
p.add_vars([theta, DistXr2Tip1,DistXr2Tip2, DistXf2Tip1,phi, currentPhi, velocity, phase, phaseOut])

## In phase 1: goal is move back to the right position
rule1 = p.rule('IF phase is one AND theta is zero AND DistXr2Tip1 is positiveFar THEN phaseOut is one AND phi is zero AND velocity is backFaster') #倒車-快
				
rule2 = p.rule('IF phase is one AND theta is zero AND DistXr2Tip1 is positiveNear THEN phaseOut is one AND phi is zero AND velocity is backSlower') #倒車-慢
				
rule3 = p.rule('IF phase is one AND theta is zero AND DistXr2Tip1 is near THEN phaseOut is two AND phi is largeNegative AND velocity is backSlower') #停止倒車-進入階段2，方向盤大右轉一些

## In phase 2, goal is to rotate wheel to negative direction until theta is 45 degree
rule4 = p.rule('IF phase is two AND theta is NOT positive45 THEN phaseOut is two AND phi is largeNegative AND velocity is backSlower') #方向盤持續右轉一些

rule5 = p.rule('IF phase is two AND theta is positive45 THEN phaseOut is three AND phi is largePositive AND velocity is zero') #停止倒車-進入階段3，方向盤迴正

## In phase 3, goal is rotate wheel back to zero 
rule6 = p.rule('IF phase is three AND currentPhi is not zero   THEN phaseOut is three AND phi is smallPositive AND velocity is zero') #倒車--慢，方向盤持續迴正

rule7 = p.rule('IF phase is three AND currentPhi is zero   THEN phaseOut is four AND phi is zero AND velocity is backSlower') #停止迴正, 倒車--慢, 進入階段4，

## In phase 4, goal is to move right back to the right position
#rule8 = p.rule('IF phase is four AND DistXr2Tip2 is positiveFar THEN phaseOut is four AND phi is zero AND velocity is backFaster') #倒車--快
rule8 = p.rule('IF phase is four AND DistXr2Tip2 is positiveFar THEN phaseOut is four AND phi is zero AND velocity is backSlower')

rule9 = p.rule('IF phase is four AND DistXr2Tip2 is positiveNear THEN phaseOut is four AND phi is zero AND velocity is backSlower') #倒車--慢

#rule10 = p.rule('IF phase is four AND DistXr2Tip2 is near THEN phaseOut is five AND phi is smallPositive AND velocity is zero') #停止倒車-進入階段5，方向盤左轉一些
rule10 = p.rule('IF phase is four AND DistXr2Tip2 is near THEN phaseOut is five AND phi is zero AND velocity is zero') 

## In phase 5, goal is to rotate handler to 45 degree
rule11 = p.rule('IF phase is five AND currentPhi is NOT smallPositive THEN phaseOut is five AND phi is smallPositive AND velocity is zero') #方向盤持續左轉一些直到 45 degree

rule12 = p.rule('IF phase is five AND currentPhi is smallPositive  THEN phaseOut is six AND phi is zero AND velocity is backSlower') #move forward

## In phase 6, goal is to move back until reach rear
rule13 = p.rule('IF phase is six AND DistXr2Tip2 is NOT VERY near  THEN phaseOut is six AND phi is zero AND velocity is backSlower') #move backward
rule14 = p.rule('IF phase is six AND DistXr2Tip2 is VERY near  THEN phaseOut is seven AND phi is zero AND velocity is zero') #get handler zero
## In phase 7, goal is rotate wheel back to negative 45 
rule15 = p.rule('IF phase is seven AND currentPhi is NOT smallNegative  THEN phaseOut is seven AND phi is smallNegative AND velocity is zero') #get handler zero

#rule16 = p.rule('IF phase is seven AND currentPhi is smallNegative  THEN phaseOut is eight AND phi is zero AND velocity is forwardSlower') #move forward
rule16 = p.rule('IF phase is seven AND currentPhi is smallNegative  THEN phaseOut is eight AND phi is smallNegative  AND velocity is forwardSlower')

## In phase 8, goal is move forwar to reach righand side of the slot
#rule17 = p.rule('IF phase is eight AND DistXf2Tip1 is NOT near    THEN phaseOut is eight AND phi is zero AND velocity is forwardSlower') #move forward
rule17 = p.rule('IF phase is eight AND theta is NOT zero THEN phaseOut is eight AND phi is smallNegative AND velocity is forwardSlower')
#rule18 = p.rule('IF phase is eight AND DistXf2Tip1 is near  THEN phaseOut is nine AND phi is zero AND velocity is zero')  #get handler zero
rule18 = p.rule('IF phase is eight AND theta is zero THEN phaseOut is nine AND phi is zero AND velocity is zero')
## In phase 9, goal is rotate wheel back to zero 
#rule19 = p.rule('IF phase is nine AND currentPhi is NOT zero   THEN phaseOut is nine AND phi is smallPositive AND velocity is zero') #get handler zero
rule19 = p.rule('IF phase is nine AND currentPhi is smallNegative THEN phaseOut is nine AND phi is smallPositive AND velocity is zero')

#rule20 = p.rule('IF phase is nine AND currentPhi is zero   THEN phaseOut is ten AND phi is zero AND velocity is zero') #stop
rule20 = p.rule('IF phase is nine AND currentPhi is zero THEN phaseOut is ten AND phi is zero AND velocity is zero')

rule21 = p.rule('IF phase is ten AND DistXf2Tip1 is negativeFar THEN phaseOut is ten AND phi is zero AND velocity is backSlower')
rule22 = p.rule('IF phase is ten AND DistXf2Tip1 is negativeNear THEN phaseOut is ten AND phi is zero AND velocity is backSlower')
rule23 = p.rule('IF phase is ten AND DistXf2Tip1 is near THEN phaseOut is eleven AND phi is zero AND velocity is zero')
#rule24 = p.rule('IF phase is ten AND currentPhi is zero AND DistXf2Tip1 is near THEN phaseOut is eleven AND phi is zero AND velocity is zero')
# rule1.view()

"""
Now that we have our rules defined, we can simply create a control system
via:
"""

parking_ctrl = ctrl.ControlSystem([rule1,rule2,rule3,rule4,rule5,rule6,rule7, rule8, rule9, rule10, rule11,
                                  rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20, rule21, rule22, rule23 ] )



"""
In order to simulate this control system, we will create a
``ControlSystemSimulation``.  Think of this object representing our controller
applied to a specific set of cirucmstances.  For tipping, this might be tipping
Sharon at the local brew-pub.  We would create another
``ControlSystemSimulation`` when we're trying to apply our ``tipping_ctrl``
for Travis at the cafe because the inputs would be different.
"""

parking = ctrl.ControlSystemSimulation(parking_ctrl)
plt.show(block=False)
plt.pause(0.1)
if __name__ == '__main__':
    simulator =  CarSim2.Simulation(400, 250) ## Draw Car Simulator
        
    # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
    # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
    th = 0 #theta
    DT1 = 0 #horizontal position 1
    DT2 = 0 #horizontal position 2
    
    ph = 1 #phase = 1 (initially)
    
    stop = False

    while (not stop) :
        # Crunch the numbers
        parking.input['theta'] = th
        parking.input['DistXr2Tip1'] = simulator.getRearToLotTip1()
        parking.input['DistXr2Tip2'] = simulator.getRearToLotTip2()
        parking.input['DistXf2Tip1'] = simulator.getFrontToLotTip1()
        parking.input['phase'] = ph
        parking.input['currentPhi'] = simulator.getCurrentPhi()
        
        print ("CAR STATE: (theta) ", th)
        print ("CAR STATE: (DistXf2Tip1) ", simulator.getFrontToLotTip1())
        print ("CAR STATE: (DistXr2Tip1) ", DT1)
        print ("CAR STATE: (DistXr2Tip2) ", DT2)
        print ("CAR STATE: (Current Phi) ", simulator.getCurrentPhi())
        print ("CAR STATE: (phase) ", ph)

        parking.compute() ## INFERENCE one step
        
        print ("RESULT: Befor adjustment (phi): ", simulator.phi)
        print ("RESULT: delta (phi) ", parking.output['phi'])
        print ("RESULT: (velocity) ", parking.output['velocity'])
        print ("RESULT: (phaseOut) ", parking.output['phaseOut'])
        
        #phi.view(sim=parking)
        #velocity.view(sim=parking)

        ## Get the new results
        
        phi = simulator.phi + parking.output['phi']
        if phi > 60*PI/180:
            phi = 60*PI/180
        elif phi < -60*PI/180:
            phi = -60*PI/180
        v= parking.output['velocity']

        simulator.drawSimulation(phi, v) # draw the system with phi, velocity
        
        ## update input data to the fuzzy system
        th = simulator.getTheta()
        DT1 = simulator.getRearToLotTip1()
        DT2 = simulator.getRearToLotTip2()
        ph_out = parking.output['phaseOut']

        if ph == 8 and abs(th) > 0.005:
            ph = 8
        else:
            ph = round(ph_out, 0)
        #if (ph==10): stop=True
        #if (ph==11): stop=True
        if ph == 10 and simulator.getFrontToLotTip1() <= -50 :
            stop = True
        elif ph == 11:
            stop = True
