from config import * # lsp-only
from driver_control import user_control # lsp-only
from autonomous import autonomous # lsp-only
# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
heightMech.close()
matchLoad.close()
descore.close()