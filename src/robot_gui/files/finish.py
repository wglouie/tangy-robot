#!python
# ---------------------------------------------------------------------------------------------
#  Python / Skype4Py example that takes a skypename from command line parameter,
#  checks if that skypename is in contact list and if yes then starts a call to that skypename.
#
#  Tested with  Skype4Py version 0.9.28.2 and Skype verson 3.5.0.214

import sys
import Skype4Py

# Creating Skype object and assigning event handlers..
skype = Skype4Py.Skype()

#skype.OnAttachmentStatus = OnAttach
#skype.OnCallStatus = OnCall

# Checking if Skype isn't running already..
if not skype.Client.IsRunning:
    print 'Desconnected'
elif skype.Client.IsRunning:
    print 'Desconnecting Skype..'
    skype.Client.Shutdown()


