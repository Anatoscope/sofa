"""a simple threaded readline console module (unix only)"""

# TODO error if not unix

from __future__ import print_function
import sys

import rlcompleter
import readline

readline.parse_and_bind("tab: complete")

def start(local = None, history = '~/.sofa_console'):
    
    import threading
    import atexit
    import os
    import code
    import sys
    import signal
    
    def target():
        
        # install readline cleanup handler
        def cleanup(): os.system('stty sane')
        atexit.register(cleanup)

        # deal with history
        if history:
            import readline
            filename = os.path.expanduser( history )
            try:
                readline.read_history_file( filename )
            except IOError as e:
                pass
                
            def write_history():
                readline.write_history_file( filename )

            atexit.register( write_history )

        namespace = local or None
        readline.set_completer(rlcompleter.Completer(namespace).complete)

        try:
            code.interact(local = local)
        except Exception as e:
            print('console error:', e)
        else:
            os.kill(os.getpid(), signal.SIGINT)
        finally:
            print('console exited')
            
        
    thread = threading.Thread(target = target)
    thread.daemon = True
    thread.start()

