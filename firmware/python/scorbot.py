import struct
from enum import IntEnum

class JointCmdOpcode(IntEnum):
    NONE = 0
    SET_PASSIVE = 1
    HOME = 2
    SET_ANGLE = 3
    SET_VELOCITY = 4
    SET_TORQUE = 5

class ScorbotCmd:
    # Format string: 6 groups of (1 byte enum + 4 byte float)
    # '<' for little-endian, 'B' for uint8 (enum), 'f' for float
    FORMAT = '<' + 'Bf' * 6
    
    def __init__(self):
        self.shoulder_pan = (JointCmdOpcode.NONE, 0.0)
        self.shoulder_lift = (JointCmdOpcode.NONE, 0.0)
        self.elbow = (JointCmdOpcode.NONE, 0.0)
        self.wrist_1 = (JointCmdOpcode.NONE, 0.0)
        self.wrist_2 = (JointCmdOpcode.NONE, 0.0)
        self.gripper = (JointCmdOpcode.NONE, 0.0)
    
    def pack(self):
        """Pack the command into a binary string"""
        return struct.pack(self.FORMAT,
            self.shoulder_pan[0], self.shoulder_pan[1],
            self.shoulder_lift[0], self.shoulder_lift[1],
            self.elbow[0], self.elbow[1],
            self.wrist_1[0], self.wrist_1[1],
            self.wrist_2[0], self.wrist_2[1],
            self.gripper[0], self.gripper[1]
        )
    
    @classmethod
    def unpack(cls, data):
        """Unpack binary data into a ScorbotCmd object"""
        cmd = cls()
        values = struct.unpack(cls.FORMAT, data)
        
        cmd.shoulder_pan = (JointCmdOpcode(values[0]), values[1])
        cmd.shoulder_lift = (JointCmdOpcode(values[2]), values[3])
        cmd.elbow = (JointCmdOpcode(values[4]), values[5])
        cmd.wrist_1 = (JointCmdOpcode(values[6]), values[7])
        cmd.wrist_2 = (JointCmdOpcode(values[8]), values[9])
        cmd.gripper = (JointCmdOpcode(values[10]), values[11])
        
        return cmd
    
    def __str__(self):
        return (
            "ScorbotCmd:\n"
            f"  Shoulder Pan: {self.shoulder_pan[0].name} = {self.shoulder_pan[1]:.2f}\n"
            f"  Shoulder Lift: {self.shoulder_lift[0].name} = {self.shoulder_lift[1]:.2f}\n"
            f"  Elbow: {self.elbow[0].name} = {self.elbow[1]:.2f}\n"
            f"  Wrist 1: {self.wrist_1[0].name} = {self.wrist_1[1]:.2f}\n"
            f"  Wrist 2: {self.wrist_2[0].name} = {self.wrist_2[1]:.2f}\n"
            f"  Gripper: {self.gripper[0].name} = {self.gripper[1]:.2f}"
        )

class ScorbotStatus:
    # Format string: 6 groups of 3 floats
    # '<' for little-endian, 'f' for float
    FORMAT = '<' + 'fff' * 6
    
    def __init__(self):
        self.shoulder_pan = (0.0, 0.0, 0.0)  # (angle, velocity, torque)
        self.shoulder_lift = (0.0, 0.0, 0.0)
        self.elbow = (0.0, 0.0, 0.0)
        self.wrist_1 = (0.0, 0.0, 0.0)
        self.wrist_2 = (0.0, 0.0, 0.0)
        self.gripper = (0.0, 0.0, 0.0)
    
    def pack(self):
        """Pack the status into a binary string"""
        return struct.pack(self.FORMAT,
            self.shoulder_pan[0], self.shoulder_pan[1], self.shoulder_pan[2],
            self.shoulder_lift[0], self.shoulder_lift[1], self.shoulder_lift[2],
            self.elbow[0], self.elbow[1], self.elbow[2],
            self.wrist_1[0], self.wrist_1[1], self.wrist_1[2],
            self.wrist_2[0], self.wrist_2[1], self.wrist_2[2],
            self.gripper[0], self.gripper[1], self.gripper[2]
        )
    
    @classmethod
    def unpack(cls, data):
        """Unpack binary data into a ScorbotStatus object"""
        status = cls()
        values = struct.unpack(cls.FORMAT, data)
        
        status.shoulder_pan = (values[0], values[1], values[2])
        status.shoulder_lift = (values[3], values[4], values[5])
        status.elbow = (values[6], values[7], values[8])
        status.wrist_1 = (values[9], values[10], values[11])
        status.wrist_2 = (values[12], values[13], values[14])
        status.gripper = (values[15], values[16], values[17])
        
        return status
    
    def __str__(self):
        return (
            "ScorbotStatus:\n"
            f"  Shoulder Pan: angle={self.shoulder_pan[0]:.2f}°, vel={self.shoulder_pan[1]:.2f}°/s, torque={self.shoulder_pan[2]:.2f}Nm\n"
            f"  Shoulder Lift: angle={self.shoulder_lift[0]:.2f}°, vel={self.shoulder_lift[1]:.2f}°/s, torque={self.shoulder_lift[2]:.2f}Nm\n"
            f"  Elbow: angle={self.elbow[0]:.2f}°, vel={self.elbow[1]:.2f}°/s, torque={self.elbow[2]:.2f}Nm\n"
            f"  Wrist 1: angle={self.wrist_1[0]:.2f}°, vel={self.wrist_1[1]:.2f}°/s, torque={self.wrist_1[2]:.2f}Nm\n"
            f"  Wrist 2: angle={self.wrist_2[0]:.2f}°, vel={self.wrist_2[1]:.2f}°/s, torque={self.wrist_2[2]:.2f}Nm\n"
            f"  Gripper: angle={self.gripper[0]:.2f}°, vel={self.gripper[1]:.2f}°/s, torque={self.gripper[2]:.2f}Nm"
        )