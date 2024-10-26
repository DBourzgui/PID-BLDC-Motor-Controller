'''
Farm PD
Driss Bourzgui - Electrical Engineering Co-Op
3/19/2024

'''

# Libraries
from machine import PWM
import micropython
import math
import time


# User interface pin defs
BT_LED = machine.Pin(0, machine.Pin.OUT)
G_LED = machine.Pin(1, machine.Pin.OUT)
R_LED = machine.Pin(14, machine.Pin.OUT)
PushButton = machine.Pin(2, machine.Pin.IN)

# Motor driver pin defs
OCM = machine.Pin(27, machine.Pin.IN)
DIAG = machine.Pin(28, machine.Pin.IN)
OCC = machine.Pin(17, machine.Pin.OUT)
OCC.value(1)
EncoderA = machine.Pin(13, machine.Pin.IN)
EncoderB = machine.Pin(12, machine.Pin.IN)
PWM1 = PWM(machine.Pin(18))
PWM2 = PWM(machine.Pin(19))
PWM1.freq(20000)
PWM2.freq(20000)
Enable = machine.Pin(22, machine.Pin.OUT)
EnableB = machine.Pin(21, machine.Pin.OUT)
Enable.value(1)
EnableB.value(0)


# Limit detection pin defs
BackLimit = machine.Pin(10, machine.Pin.IN)
ForwardLimit = machine.Pin(11, machine.Pin.IN)
CutLimit = machine.Pin(8, machine.Pin.IN)


# Sequence indexing vars
SequenceState = 0

# Motor parameter vars
Encoder_SINGCHAN_CPR = 48
GearBoxRatio = 20.4
Encoder_CPR_Out = Encoder_SINGCHAN_CPR * GearBoxRatio #12 CPR for single encoder channel * 20.4:1 gearbox ratio

# Encoder interrupt handler vars
TimePrev_INT = 0
TimePrev = 0
Position = 0
Velocity = 0
CapturedPosi = 0
CapturedVel = 0

# Velocity readout filtering vars
Alpha = 0.85
Beta = (1 - Alpha)
FilteredVel = 0
NewVel = 0

# PID control vars
AlphaPID = 0.85
BetaPID = (1 - AlphaPID)
PrevPWM = 0
NewPWM = 0
TimePrev = 0
ErrInteg = 0
ErrPrev = 0
PID_Sign = 1
ErrSign = 1
SignFlag = 0
kp = 139.51 
kd = 15 # Needs Tuning
ki = 0.1 # Needs Tuning
Overshoot = 0
RUN = 0

# OLED vars
DeployInc = machine.Pin(15, machine.Pin.OUT)

# Button press handler
def ButtonPress(p):
    global SequenceState
    print("Pressed")
    if RUN == 0:
        SequenceState += 1
        
PushButton.irq(trigger=PushButton.IRQ_FALLING, handler=ButtonPress)

# Motor driver error detection handler
def DiagnosticFlag(p):
    PWM1.duty_u16(0)
    PWM2.duty_u16(0)
    R_LED.value(1)
    print("ERROR DETECTED")

DIAG.irq(trigger=DIAG.IRQ_FALLING, handler=DiagnosticFlag)

# Motor encoder handler
def EncoderReadOut(p):
    global Position
    global Direction
    global DeltaT_INT
    global TimePrev_INT
    global Velocity
    if EncoderB.value():
        Position += 1
    else:
        Position -= 1
        
    CurrentTime_INT = time.ticks_us()
    DeltaT_INT = time.ticks_diff(CurrentTime_INT, TimePrev_INT)/1e6
    Velocity = 1/DeltaT_INT
    TimePrev_INT = CurrentTime_INT
 
EncoderA.irq(trigger=machine.Pin.IRQ_RISING, handler=EncoderReadOut)


# PID motor speed controller function 
def PID(Target, Direction):
    
    global TimePrev, Position, CapturedPosi, Velocity, CapturedVel, FilteredVel, Alpha, Beta, Encoder_CPR_Out, ErrPrev, ErrInteg, kp, ki, kd, PrevPWM, SignFlag, PeakFlag

    if Direction == 1:
        PWM_Engaged = PWM2
        PWM_Halted = PWM1
    elif Direction == -1:
        PWM_Engaged = PWM1
        PWM_Halted = PWM2
        
    CurrentTime = time.ticks_us()
    DeltaT = time.ticks_diff(CurrentTime, TimePrev)/1e6
    TimePrev = CurrentTime
    
    # May lose encoder counts at high RPMs from following IRQ disable block (~30us):
    State = machine.disable_irq()
    CapturedPosi = Position
    CapturedVel = Velocity
    machine.enable_irq(State)
    
   
    NewVel = CapturedVel
    FilteredVel = PseudoLowPass(FilteredVel, NewVel, AlphaPID, BetaPID)
    VelocityRPM = FilteredVel/Encoder_CPR_Out*60
    
    Error = VelocityRPM - Target
    ErrDeriv = (Error - ErrPrev)/DeltaT
    ErrInteg = ErrInteg + (Error * DeltaT) #  First order finite estimation for integral calc.
    PID = (kp * Error) + (ki * ErrInteg) + (kd * ErrDeriv)
    ErrPrev = Error
    
    pwr = (abs(PID))
    
    PID_Sign = math.copysign(1, PID)
    ErrSign = math.copysign(1, Error)
    
    if (ErrSign == PID_Sign):
        SignFlag = 1
    else:
        SignFlag = 0
    
    if (PID < 0):
        NewPWM = int(PrevPWM + pwr)
        if (NewPWM > 65536): # Clamping saturation limit (max. bound)
            NewPWM = 65536
            PeakFlag = 1
        else:
            PeakFlag = 0
            
        if (NewPWM == PrevPWM) & (PeakFlag == 1) & (SignFlag == 1):    # Anti-windup control 
            NewPWM = int(PrevPWM + abs((kp * Error) + (kd * ErrDeriv)))
            ErrInteg = ErrInteg - (Error * DeltaT)
            WindupFlag = 1
        else:
            WindupFlag = 0
            
        PWM_Engaged.duty_u16(NewPWM)
        PWM_Halted.duty_u16(0)
    elif (PID > 0):
        NewPWM = int(PrevPWM - pwr)
        if (NewPWM < 0): # Clamping saturation limit (min. bound)
            NewPWM = 0
        PWM_Engaged.duty_u16(NewPWM)
        PWM_Halted.duty_u16(0)
    PrevPWM = NewPWM

    #print(CapturedPosi, Target)

# PID variable reset function
def PID_Reset(p):
    TimePrev = 0
    Velocity = 0
    CapturedVel = 0
    FilteredVel = 0
    PrevPWM = 0
    NewPWM = 0
    TimePrev = 0
    ErrInteg = 0
    ErrPrev = 0

# Simplified low-pass filter function 
def PseudoLowPass(PrevVal, NewVal, Alpha, Beta):
    return Alpha * PrevVal + Beta * NewVal

# Main sequence loop
while True:
    
    if DIAG.value() == 0:
        R_LED.value(1)
    else:
        R_LED.value(0)
    
    # Wait-to-start sequence
    if SequenceState == 0:
        CapturedPosi = 0
        Position = 0
        PWM1.duty_u16(0)
        PWM2.duty_u16(0)
        
        print("START SEQUENCE")
        print("Waiting...")
        
        BT_LED.value(1)
        G_LED.value(1)
        time.sleep(0.5)
        BT_LED.value(0)
        G_LED.value(0)
        time.sleep(0.5)
        
    # Run sequence        
    else:
        
        # First forward movement 
        RUN = 1
        
        while ((CapturedPosi > -2200) & (SequenceState == 1) & (ForwardLimit.value() == 0)):
            PID(20, -1)
        Posi1 = -2200
        PWM1.duty_u16(0)
        PWM2.duty_u16(0)
        T_End = time.time() + 2
        while (time.time() < T_End):
            CapturedPosi = Position
        Overshoot1 = Posi1 - CapturedPosi
        print ("Position:", CapturedPosi, "Overshoot:", Overshoot1)
        PID_Reset(1)

        # Return-to-home movement
        while ((CapturedPosi < 0) & (SequenceState == 1) & (BackLimit.value() == 0)):  
            PID(20, 1)
        PWM1.duty_u16(0)
        PWM2.duty_u16(0)
        T_End = time.time() + 2
        while (time.time() < T_End):
            CapturedPosi = Position
        Overshoot = CapturedPosi
        print ("Position:", CapturedPosi, "Overshoot:", Overshoot)
        
        PID_Reset(1)
        SequenceState = 0
        
        # OLED display deployment value increment 
        DeployInc.value(1)
        time.sleep(0.25)
        DeployInc.value(0)
        
        RUN = 0