import GameLogic
import math
import morse.core.actuator
import morse.helpers.math as morse_math

class KukaIKActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller for the Kuka LWR arm

    This component will read an array of 7 floats, and apply them as
    rotation for the parts of the Kuka arm.
    """

    def __init__(self, obj, parent=None):
        print ('######## KUKA CONTROL INITIALIZATION ########')
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._speed = self.blender_obj['Speed']
        self._tolerance = math.radians(0.5)

        self.local_data['seg0'] = 0.0
        self.local_data['seg1'] = 0.0
        self.local_data['seg2'] = 0.0
        self.local_data['seg3'] = 0.0
        self.local_data['seg4'] = 0.0
        self.local_data['seg5'] = 0.0
        self.local_data['seg6'] = 0.0

        # The axis along which the different segments rotate
        # Considering the rotation of the arm as installed in Jido
        self._dofs = ['z', 'y', 'z', 'y', 'z', 'y', 'z']

        print ('######## KUKA CONTROL INITIALIZED ########')



    def default_action(self):
        """ Apply rotation angles to the segments of the arm """

        # Reset movement variables
        rx, ry, rz = 0.0, 0.0, 0.0

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardles of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        ticks = GameLogic.getLogicTicRate()
        # Scale the speeds to the time used by Blender
        try:
            rotation = self._speed / ticks
        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass

        armature = self.blender_obj
        print ("The armature is: '%s' (%s)" % (armature, type(armature)))

        i = 0
        for channel in armature.channels:
            segment_angle = channel.joint_rotation
            #print ("\tChannel '%s': (%.4f, %.4f, %.4f)" % (channel, segment_angle[0], segment_angle[1], segment_angle[2]))

            key = ('seg%d' % i)
            # Get the normalised angle for this segment
            target_angle = morse_math.normalise_angle(self.local_data[key])
            print ("%.4f " % target_angle, end='')

            # Use the corresponding direction for each rotation
            if self._dofs[i] == 'y':
                ry = morse_math.rotation_direction(segment_angle[1], target_angle, self._tolerance, rotation)
            elif self._dofs[i] == 'z':
                rz = morse_math.rotation_direction(segment_angle[2], target_angle, self._tolerance, rotation)

            print ("[%.4f, %.4f, %.4f] " % (rx, ry, rz))

            # Give the movement instructions directly to the parent
            # The second parameter specifies a "local" movement
            #segment.applyRotation([rx, ry, rz], True)
            channel.joint_rotation = [rx, ry, rz]

            # Reset the rotations for the next segment
            ry = rz = 0
            i = i + 1
