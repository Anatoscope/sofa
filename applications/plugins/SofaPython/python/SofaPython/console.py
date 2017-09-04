"""a simple threaded readline console module (unix only)"""

def start(local = None, history = None):
    
    import threading
    import atexit
    import os
    import code
    
    def target():
        
        # install readline cleanup handler
        def cleanup(): os.system('stty sane')
        atexit.register(cleanup)

        # deal with history
        if history:
            import readline
            filename = os.path.expanduser( history )
            readline.read_history_file( filename )

            def write_history():
                readline.write_history_file( filename )

            atexit.register( write_history )
            
        code.interact(local = local)

    thread = threading.Thread(target = target)
    thread.daemon = True
    thread.start()

