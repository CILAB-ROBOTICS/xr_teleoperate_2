from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
#from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
#from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array, Lock
from inspire_sdkpy import inspire_dds # lazy import
import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default

import logging_mp
logger_mp = logging_mp.get_logger(__name__)


inspire_tip_indices = [4, 9, 14, 19, 24]
Inspire_Num_Motors = 6
kTopicInspireDFXCommand = "rt/inspire/cmd"
kTopicInspireDFXState = "rt/inspire/state"
'''
class Inspire_Controller_DFX:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, dual_hand_touch_array = None, dual_hand_force_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False):
        logger_mp.info("Initialize Inspire_Controller_DFX...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # initialize handcmd publisher and handstate subscriber
        self.HandCmb_publisher = ChannelPublisher(kTopicInspireDFXCommand, MotorCmds_)
        self.HandCmb_publisher.Init()

        self.HandState_subscriber = ChannelSubscriber(kTopicInspireDFXState, MotorStates_)
        self.HandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)  
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        self.left_tactile_array = Array('d', Inspire_Num_Tactile, lock=True)
        self.right_tactile_array = Array('d', Inspire_Num_Tactile, lock=True)
        
        self.left_hand_force_array = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_force_array = Array('d', Inspire_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        while True:
            if any(self.right_hand_state_array): # any(self.left_hand_state_array) and 
                break
            time.sleep(0.01)
            logger_mp.warning("[Inspire_Controller_DFX] Waiting to subscribe dds...")
        logger_mp.info("[Inspire_Controller_DFX] Subscribe dds ok.")

        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array, self.left_hand_state_array, self.right_hand_state_array,
                                                                          self.left_tactile_array, self.right_tactile_array, self.left_hand_force_array, self.right_hand_force_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, dual_hand_touch_array, dual_hand_force_array))
        
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_DFX OK!")

    def _subscribe_hand_state(self):
        while True:
            hand_msg  = self.HandState_subscriber.Read()
            if hand_msg is not None:
                for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
                    self.left_hand_state_array[idx] = hand_msg.states[id].q
                for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
                    self.right_hand_state_array[idx] = hand_msg.states[id].q
            time.sleep(0.002)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = left_q_target[idx]         
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = right_q_target[idx] 

        self.HandCmb_publisher.Write(self.hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")

    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                        left_hand_touch_array, shared_right_hand_touch_array, left_hand_force_array, right_hand_force_array,
                        dual_hand_data_lock=None, dual_hand_state_array=None, dual_hand_action_array=None, dual_hand_touch_array=None, dual_hand_force_array=None):
        self.running = True

        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        # initialize inspire hand's cmd msg
        self.hand_msg  = MotorCmds_()
        self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_Right_Hand_JointIndex) + len(Inspire_Left_Hand_JointIndex))]

        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                    #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                    # The q_target now is in radians, ranges:
                    #     - idx 0~3: 0~1.7 (1.7 = closed)
                    #     - idx 4:   0~0.5
                    #     - idx 5:  -0.1~1.3
                    # We normalize them using (max - value) / range
                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))    
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_DFX has been closed.")
'''


kTopicInspireFTPLeftCommand   = "rt/inspire_hand/ctrl/l"
kTopicInspireFTPRightCommand  = "rt/inspire_hand/ctrl/r"
kTopicInspireFTPLeftState  = "rt/inspire_hand/state/l"
kTopicInspireFTPRightState = "rt/inspire_hand/state/r"
kTopicInspireTouchLeft = "rt/inspire_hand/touch/l"
kTopicInspireTouchRight = "rt/inspire_hand/touch/r"

touch_dict = {
    "fingerone_tip_touch": 9,
    "fingerone_top_touch": 96,
    "fingerone_palm_touch": 80,
    "fingertwo_tip_touch": 9,
    "fingertwo_top_touch": 96,
    "fingertwo_palm_touch": 80,
    "fingerthree_tip_touch": 9,
    "fingerthree_top_touch": 96,
    "fingerthree_palm_touch": 80,
    "fingerfour_tip_touch": 9,
    "fingerfour_top_touch": 96,
    "fingerfour_palm_touch": 80,
    "fingerfive_tip_touch": 9,
    "fingerfive_top_touch": 96,
    "fingerfive_middle_touch": 9,
    "fingerfive_palm_touch": 96,
    "palm_touch": 112
}
Inspire_Num_Tactile = 1062

alpha = 0.1  
Kp = 0.4 
Kd = 0.02 

class Inspire_Controller_FTP:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, dual_hand_touch_array = None, dual_hand_force_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False):
        logger_mp.info("Initialize Inspire_Controller_FTP...")
        # from inspire_sdkpy import inspire_dds  # lazy import
        # import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.debug_counter = 0
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # Initialize hand command publishers
        self.LeftHandCmd_publisher = ChannelPublisher(kTopicInspireFTPLeftCommand, inspire_dds.inspire_hand_ctrl)
        self.LeftHandCmd_publisher.Init()
        self.RightHandCmd_publisher = ChannelPublisher(kTopicInspireFTPRightCommand, inspire_dds.inspire_hand_ctrl)
        self.RightHandCmd_publisher.Init()

        # Initialize hand state subscribers
        self.LeftHandState_subscriber = ChannelSubscriber(kTopicInspireFTPLeftState, inspire_dds.inspire_hand_state)
        self.LeftHandState_subscriber.Init() # Consider using callback if preferred: Init(callback_func, period_ms)
        self.RightHandState_subscriber = ChannelSubscriber(kTopicInspireFTPRightState, inspire_dds.inspire_hand_state)
        self.RightHandState_subscriber.Init()

        self.LeftTouch_subscriber = ChannelSubscriber(kTopicInspireTouchLeft, inspire_dds.inspire_hand_touch)
        self.LeftTouch_subscriber.Init()
        self.RightTouch_subscriber = ChannelSubscriber(kTopicInspireTouchRight, inspire_dds.inspire_hand_touch)
        self.RightTouch_subscriber.Init()

        # Shared Arrays for hand states ([0,1] normalized values)
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        self.left_tactile_array = Array('d', Inspire_Num_Tactile, lock=True)
        self.right_tactile_array = Array('d', Inspire_Num_Tactile, lock=True)
        
        self.left_hand_force_array = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_force_array = Array('d', Inspire_Num_Motors, lock=True)

        # Initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Wait for initial DDS messages (optional, but good for ensuring connection)
        wait_count = 0
        while not (any(self.left_hand_state_array) or any(self.right_hand_state_array)):
            if wait_count % 100 == 0: # Print every second
                logger_mp.info(f"[Inspire_Controller_FTP] Waiting to subscribe to hand states from DDS (L: {any(self.left_hand_state_array)}, R: {any(self.right_hand_state_array)})...")
            time.sleep(0.01)
            wait_count += 1
            if wait_count > 500: # Timeout after 5 seconds
                logger_mp.warning("[Inspire_Controller_FTP] Warning: Timeout waiting for initial hand states. Proceeding anyway.")
                break
        logger_mp.info("[Inspire_Controller_FTP] Initial hand states received or timeout.")

        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array, self.left_hand_state_array, self.right_hand_state_array,
                                                                          self.left_tactile_array, self.right_tactile_array, self.left_hand_force_array, self.right_hand_force_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, dual_hand_touch_array, dual_hand_force_array))
        
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_FTP OK!\n")

        self._touch_offsets = {}
        start = 0
        for name, length in touch_dict.items(): 
            self._touch_offsets[name] = (start, start + length)
            start += length  
   
    def _subscribe_hand_state(self):
        logger_mp.info("[Inspire_Controller_FTP] Subscribe thread started.")
        while True:
            # Left Hand
            left_state_msg = self.LeftHandState_subscriber.Read()
            if left_state_msg is not None:
                if hasattr(left_state_msg, 'angle_act') and len(left_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.left_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.left_hand_state_array[i] = left_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received left_state_msg but attributes are missing or incorrect. Type: {type(left_state_msg)}, Content: {str(left_state_msg)[:100]}")
            # Right Hand
            right_state_msg = self.RightHandState_subscriber.Read()
            if right_state_msg is not None:
                if hasattr(right_state_msg, 'angle_act') and len(right_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.right_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.right_hand_state_array[i] = right_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received right_state_msg but attributes are missing or incorrect. Type: {type(right_state_msg)}, Content: {str(right_state_msg)[:100]}")

            # Left Tactile
            left_touch_msg = self.LeftTouch_subscriber.Read()
            if left_touch_msg is not None:
                if hasattr(left_touch_msg, 'fingerone_tip_touch') and len(left_touch_msg.fingerone_tip_touch) == 9:
                    with self.left_tactile_array.get_lock():
                        for i in range(9):
                            self.left_tactile_array[i] = left_touch_msg.fingerone_tip_touch[i]

                if hasattr(left_touch_msg, 'fingerone_top_touch') and len(left_touch_msg.fingerone_top_touch) == 96:
                    with self.left_tactile_array.get_lock():
                        for i in range(96):
                            self.left_tactile_array[9 + i] = left_touch_msg.fingerone_top_touch[i]

                if hasattr(left_touch_msg, 'fingerone_palm_touch') and len(left_touch_msg.fingerone_palm_touch) == 80:
                    with self.left_tactile_array.get_lock():
                        for i in range(80):
                            self.left_tactile_array[105 + i] = left_touch_msg.fingerone_palm_touch[i]

                if hasattr(left_touch_msg, 'fingertwo_tip_touch') and len(left_touch_msg.fingertwo_tip_touch) == 9:
                    with self.left_tactile_array.get_lock():
                        for i in range(9):
                            self.left_tactile_array[185 + i] = left_touch_msg.fingertwo_tip_touch[i]

                if hasattr(left_touch_msg, 'fingertwo_top_touch') and len(left_touch_msg.fingertwo_top_touch) == 96:
                    with self.left_tactile_array.get_lock():
                        for i in range(96):
                            self.left_tactile_array[194 + i] = left_touch_msg.fingertwo_top_touch[i]

                if hasattr(left_touch_msg, 'fingertwo_palm_touch') and len(left_touch_msg.fingertwo_palm_touch) == 80:
                    with self.left_tactile_array.get_lock():
                        for i in range(80):
                            self.left_tactile_array[290 + i] = left_touch_msg.fingertwo_palm_touch[i]

                if hasattr(left_touch_msg, 'fingerthree_tip_touch') and len(left_touch_msg.fingerthree_tip_touch) == 9:
                    with self.left_tactile_array.get_lock():
                        for i in range(9):
                            self.left_tactile_array[370 + i] = left_touch_msg.fingerthree_tip_touch[i]

                if hasattr(left_touch_msg, 'fingerthree_top_touch') and len(left_touch_msg.fingerthree_top_touch) == 96:
                    with self.left_tactile_array.get_lock():
                        for i in range(96):
                            self.left_tactile_array[379 + i] = left_touch_msg.fingerthree_top_touch[i]

                if hasattr(left_touch_msg, 'fingerthree_palm_touch') and len(left_touch_msg.fingerthree_palm_touch) == 80:
                    with self.left_tactile_array.get_lock():
                        for i in range(80):
                            self.left_tactile_array[475 + i] = left_touch_msg.fingerthree_palm_touch[i]

                if hasattr(left_touch_msg, 'fingerfour_tip_touch') and len(left_touch_msg.fingerfour_tip_touch) == 9:
                    with self.left_tactile_array.get_lock():
                        for i in range(9):
                            self.left_tactile_array[555 + i] = left_touch_msg.fingerfour_tip_touch[i]

                if hasattr(left_touch_msg, 'fingerfour_top_touch') and len(left_touch_msg.fingerfour_top_touch) == 96:
                    with self.left_tactile_array.get_lock():
                        for i in range(96):
                            self.left_tactile_array[564 + i] = left_touch_msg.fingerfour_top_touch[i]

                if hasattr(left_touch_msg, 'fingerfour_palm_touch') and len(left_touch_msg.fingerfour_palm_touch) == 80:
                    with self.left_tactile_array.get_lock():
                        for i in range(80):
                            self.left_tactile_array[660 + i] = left_touch_msg.fingerfour_palm_touch[i]

                if hasattr(left_touch_msg, 'fingerfive_tip_touch') and len(left_touch_msg.fingerfive_tip_touch) == 9:
                    with self.left_tactile_array.get_lock():
                        for i in range(9):
                            self.left_tactile_array[740 + i] = left_touch_msg.fingerfive_tip_touch[i]

                if hasattr(left_touch_msg, 'fingerfive_top_touch') and len(left_touch_msg.fingerfive_top_touch) == 96:
                    with self.left_tactile_array.get_lock():
                        for i in range(96):
                            self.left_tactile_array[749 + i] = left_touch_msg.fingerfive_top_touch[i]

                if hasattr(left_touch_msg, 'fingerfive_middle_touch') and len(left_touch_msg.fingerfive_middle_touch) == 9:
                    with self.left_tactile_array.get_lock():
                        for i in range(9):
                            self.left_tactile_array[845 + i] = left_touch_msg.fingerfive_middle_touch[i]

                if hasattr(left_touch_msg, 'fingerfive_palm_touch') and len(left_touch_msg.fingerfive_palm_touch) == 96:
                    with self.left_tactile_array.get_lock():
                        for i in range(96):
                            self.left_tactile_array[854 + i] = left_touch_msg.fingerfive_palm_touch[i]

                if hasattr(left_touch_msg, 'palm_touch') and len(left_touch_msg.palm_touch) == 112:
                    with self.left_tactile_array.get_lock():
                        for i in range(112):
                            self.left_tactile_array[950 + i] = left_touch_msg.palm_touch[i]  

            # Right Tactile
            right_touch_msg = self.RightTouch_subscriber.Read()
            if right_touch_msg is not None:          
                if hasattr(right_touch_msg, 'fingerone_tip_touch') and len(right_touch_msg.fingerone_tip_touch) == 9:
                    with self.right_tactile_array.get_lock():
                        for i in range(9):
                            self.right_tactile_array[i] = right_touch_msg.fingerone_tip_touch[i]

                if hasattr(right_touch_msg, 'fingerone_top_touch') and len(right_touch_msg.fingerone_top_touch) == 96:
                    with self.right_tactile_array.get_lock():
                        for i in range(96):
                            self.right_tactile_array[9 + i] = right_touch_msg.fingerone_top_touch[i]

                if hasattr(right_touch_msg, 'fingerone_palm_touch') and len(right_touch_msg.fingerone_palm_touch) == 80:
                    with self.right_tactile_array.get_lock():
                        for i in range(80):
                            self.right_tactile_array[105 + i] = right_touch_msg.fingerone_palm_touch[i]

                if hasattr(right_touch_msg, 'fingertwo_tip_touch') and len(right_touch_msg.fingertwo_tip_touch) == 9:
                    with self.right_tactile_array.get_lock():
                        for i in range(9):
                            self.right_tactile_array[185 + i] = right_touch_msg.fingertwo_tip_touch[i]

                if hasattr(right_touch_msg, 'fingertwo_top_touch') and len(right_touch_msg.fingertwo_top_touch) == 96:
                    with self.right_tactile_array.get_lock():
                        for i in range(96):
                            self.right_tactile_array[194 + i] = right_touch_msg.fingertwo_top_touch[i]

                if hasattr(right_touch_msg, 'fingertwo_palm_touch') and len(right_touch_msg.fingertwo_palm_touch) == 80:
                    with self.right_tactile_array.get_lock():
                        for i in range(80):
                            self.right_tactile_array[290 + i] = right_touch_msg.fingertwo_palm_touch[i]

                if hasattr(right_touch_msg, 'fingerthree_tip_touch') and len(right_touch_msg.fingerthree_tip_touch) == 9:
                    with self.right_tactile_array.get_lock():
                        for i in range(9):
                            self.right_tactile_array[370 + i] = right_touch_msg.fingerthree_tip_touch[i]

                if hasattr(right_touch_msg, 'fingerthree_top_touch') and len(right_touch_msg.fingerthree_top_touch) == 96:
                    with self.right_tactile_array.get_lock():
                        for i in range(96):
                            self.right_tactile_array[379 + i] = right_touch_msg.fingerthree_top_touch[i]

                if hasattr(right_touch_msg, 'fingerthree_palm_touch') and len(right_touch_msg.fingerthree_palm_touch) == 80:
                    with self.right_tactile_array.get_lock():
                        for i in range(80):
                            self.right_tactile_array[475 + i] = right_touch_msg.fingerthree_palm_touch[i]

                if hasattr(right_touch_msg, 'fingerfour_tip_touch') and len(right_touch_msg.fingerfour_tip_touch) == 9:
                    with self.right_tactile_array.get_lock():
                        for i in range(9):
                            self.right_tactile_array[555 + i] = right_touch_msg.fingerfour_tip_touch[i]

                if hasattr(right_touch_msg, 'fingerfour_top_touch') and len(right_touch_msg.fingerfour_top_touch) == 96:
                    with self.right_tactile_array.get_lock():
                        for i in range(96):
                            self.right_tactile_array[564 + i] = right_touch_msg.fingerfour_top_touch[i]

                if hasattr(right_touch_msg, 'fingerfour_palm_touch') and len(right_touch_msg.fingerfour_palm_touch) == 80:
                    with self.right_tactile_array.get_lock():
                        for i in range(80):
                            self.right_tactile_array[660 + i] = right_touch_msg.fingerfour_palm_touch[i]

                if hasattr(right_touch_msg, 'fingerfive_tip_touch') and len(right_touch_msg.fingerfive_tip_touch) == 9:
                    with self.right_tactile_array.get_lock():
                        for i in range(9):
                            self.right_tactile_array[740 + i] = right_touch_msg.fingerfive_tip_touch[i]

                if hasattr(right_touch_msg, 'fingerfive_top_touch') and len(right_touch_msg.fingerfive_top_touch) == 96:
                    with self.right_tactile_array.get_lock():
                        for i in range(96):
                            self.right_tactile_array[749 + i] = right_touch_msg.fingerfive_top_touch[i]

                if hasattr(right_touch_msg, 'fingerfive_middle_touch') and len(right_touch_msg.fingerfive_middle_touch) == 9:
                    with self.right_tactile_array.get_lock():
                        for i in range(9):
                            self.right_tactile_array[845 + i] = right_touch_msg.fingerfive_middle_touch[i]

                if hasattr(right_touch_msg, 'fingerfive_palm_touch') and len(right_touch_msg.fingerfive_palm_touch) == 96:
                    with self.right_tactile_array.get_lock():
                        for i in range(96):
                            self.right_tactile_array[854 + i] = right_touch_msg.fingerfive_palm_touch[i]

                if hasattr(right_touch_msg, 'palm_touch') and len(right_touch_msg.palm_touch) == 112:
                    with self.right_tactile_array.get_lock():
                        for i in range(112):
                            self.right_tactile_array[950 + i] = right_touch_msg.palm_touch[i] 

            time.sleep(0.002)

    def _send_hand_command(self, left_angle_cmd_scaled, right_angle_cmd_scaled, left_force_cmd_scaled, right_force_cmd_scaled, left_speed_cmd_scaled, right_speed_cmd_scaled):                           
        """
        Send scaled angle commands [0-1000] to both hands.
        """
        # Left Hand Command
        left_cmd_msg = inspire_hand_default.get_inspire_hand_ctrl()
        left_cmd_msg.angle_set = left_angle_cmd_scaled
        left_cmd_msg.force_set = left_force_cmd_scaled
        left_cmd_msg.speed_set = left_speed_cmd_scaled
        left_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.LeftHandCmd_publisher.Write(left_cmd_msg)

        # Right Hand Command
        right_cmd_msg = inspire_hand_default.get_inspire_hand_ctrl()
        right_cmd_msg.angle_set = right_angle_cmd_scaled
        right_cmd_msg.force_set = right_force_cmd_scaled
        right_cmd_msg.speed_set = right_speed_cmd_scaled
        right_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.RightHandCmd_publisher.Write(right_cmd_msg)

        # 临时打开前 N 次的 log
        if not hasattr(self, "_debug_count"):
            self._debug_count = 0
        if self._debug_count < 50:
            logger_mp.info(f"[Inspire_Controller_FTP] Publish cmd L={left_angle_cmd_scaled} R={right_angle_cmd_scaled} ")
            self._debug_count += 1


    def control_process(self, left_hand_array, right_hand_array, 
                        left_hand_state_array, right_hand_state_array,
                        shared_left_hand_touch_array, shared_right_hand_touch_array,
                        shared_left_hand_force_array, shared_right_hand_force_array, 
                        dual_hand_data_lock = None, 
                        dual_hand_state_array = None, dual_hand_action_array = None,
                        dual_hand_touch_array_shm=None, dual_hand_force_array_shm=None):
        logger_mp.info("[Inspire_Controller_FTP] Control process started.")
        self.running = True

        #left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        #right_q_target = np.full(Inspire_Num_Motors, 1.0)
        current_left_q_target_norm = np.ones(Inspire_Num_Motors, dtype=float) 
        current_right_q_target_norm = np.ones(Inspire_Num_Motors, dtype=float)
        prev_left_angle_norm = current_left_q_target_norm.copy()
        prev_right_angle_norm = current_right_q_target_norm.copy()
        prev_err_left  = np.zeros(6)
        prev_err_right = np.zeros(6)

        left_q_target = np.ones(Inspire_Num_Motors)
        right_q_target = np.ones(Inspire_Num_Motors)

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                with left_hand_state_array.get_lock():
                    current_left_q_state_norm = np.array(left_hand_state_array[:])
                with right_hand_state_array.get_lock():
                    current_right_q_state_norm = np.array(right_hand_state_array[:])

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                with shared_left_hand_touch_array.get_lock():
                    current_left_q_touch_norm = np.array(shared_left_hand_touch_array[:])
                with shared_right_hand_touch_array.get_lock():
                    current_right_q_touch_norm = np.array(shared_right_hand_touch_array[:])
                touch_data = np.concatenate((current_left_q_touch_norm, current_right_q_touch_norm))

                with shared_left_hand_force_array.get_lock():
                    current_left_q_force_norm = np.array(shared_left_hand_force_array[:])
                with shared_right_hand_force_array.get_lock():
                    current_right_q_force_norm = np.array(shared_right_hand_force_array[:])    
                force_data = np.concatenate((current_left_q_force_norm, current_right_q_force_norm))
                
                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            #left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            #right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                            min_r, max_r = 0.0, 1.7 # Retargeting output: 0.0 (open) to 1.7 (closed)
                        elif idx == 4:
                            #left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            #right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                            min_r, max_r = 0.0, 0.5  # Retargeting output: 0.0 (open) to 0.5 (closed)
                        elif idx == 5:
                            #left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            #right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)
                            min_r, max_r = -0.1, 1.3 # Retargeting output: -0.1 ( adduct? open-ish) to 1.3 (abduct? closed-ish)

                        current_left_q_target_norm[idx] = normalize(left_q_target[idx], min_r, max_r)
                        current_right_q_target_norm[idx] = normalize(right_q_target[idx], min_r, max_r)

                scaled_left_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in current_left_q_target_norm]
                scaled_right_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in current_right_q_target_norm]

                dt = 1.0 / self.fps
                raw_left_speed  = (current_left_q_target_norm  - prev_left_angle_norm)  / dt
                raw_right_speed = (current_right_q_target_norm - prev_right_angle_norm) / dt
                
                left_speed_cmd_scaled  = np.clip(raw_left_speed,  0, 1) * 1000
                right_speed_cmd_scaled = np.clip(raw_right_speed, 0, 1) * 1000
                
                left_speed_cmd_scaled  = left_speed_cmd_scaled.astype(int).tolist()
                right_speed_cmd_scaled = right_speed_cmd_scaled.astype(int).tolist()
                
                prev_left_angle_norm  = current_left_q_target_norm.copy()
                prev_right_angle_norm = current_right_q_target_norm.copy()
                
                err_left  = current_left_q_target_norm  - current_left_q_state_norm 
                err_right = current_right_q_target_norm - current_right_q_state_norm

                d_err_left  = (err_left  - prev_err_left)  / dt
                d_err_right = (err_right - prev_err_right) / dt

                u_left  = Kp * err_left  + Kd * d_err_left
                u_right = Kp * err_right + Kd * d_err_right

                desired_force_left  = np.clip(u_left,  0, 1) 
                desired_force_right = np.clip(u_right, 0, 1)

                left_force_cmd_scaled  = (u_left  * 3000).astype(int).tolist()
                right_force_cmd_scaled = (u_right * 3000).astype(int).tolist()

                prev_err_left  = err_left
                prev_err_right = err_right

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target, desired_force_left, desired_force_right, raw_left_speed, raw_right_speed,))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                if dual_hand_force_array_shm and dual_hand_data_lock:
                    with dual_hand_data_lock:
                        dual_hand_force_array_shm[:] = force_data 

                if dual_hand_touch_array_shm and dual_hand_data_lock:
                    with dual_hand_data_lock:
                        dual_hand_touch_array_shm[:] = touch_data

                self._send_hand_command(scaled_left_cmd, scaled_right_cmd, left_force_cmd_scaled, right_force_cmd_scaled, left_speed_cmd_scaled, right_speed_cmd_scaled)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_FTP has been closed.")

# Update hand state, according to the official documentation:
# 1. https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand
# 2. https://support.unitree.com/home/en/G1_developer/inspire_ftp_dexterity_hand
# the state sequence is as shown in the table below
# ┌──────┬───────┬──────┬────────┬────────┬────────────┬────────────────┬───────┬──────┬────────┬────────┬────────────┬────────────────┐
# │ Id   │   0   │  1   │   2    │   3    │     4      │       5        │   6   │  7   │   8    │   9    │    10      │       11       │
# ├──────┼───────┼──────┼────────┼────────┼────────────┼────────────────┼───────┼──────┼────────┼────────┼────────────┼────────────────┤
# │      │                    Right Hand                                │                   Left Hand                                  │
# │Joint │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │
# └──────┴───────┴──────┴────────┴────────┴────────────┴────────────────┴───────┴──────┴────────┴────────┴────────────┴────────────────┘
class Inspire_Right_Hand_JointIndex(IntEnum):
    kRightHandPinky = 0
    kRightHandRing = 1
    kRightHandMiddle = 2
    kRightHandIndex = 3
    kRightHandThumbBend = 4
    kRightHandThumbRotation = 5

class Inspire_Left_Hand_JointIndex(IntEnum):
    kLeftHandPinky = 6
    kLeftHandRing = 7
    kLeftHandMiddle = 8
    kLeftHandIndex = 9
    kLeftHandThumbBend = 10
    kLeftHandThumbRotation = 11
