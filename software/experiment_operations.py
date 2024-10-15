class ExperimentOperations:
    def __init__(self, fluid_controller, syringe_pump, config, mapping):
        self.fc = fluid_controller
        self.sp = syringe_pump
        self.config = config
        self.simplified_to_actual = mapping
        self.progress_callback = None
        self.error_callback = None
        self.abort_requested = False

    def set_callbacks(self, progress_callback, error_callback):
        self.progress_callback = progress_callback
        self.error_callback = error_callback

    def update_progress(self, message, percentage=None):
        if self.progress_callback:
            self.progress_callback(message, percentage)

    def report_error(self, error_message):
        if self.error_callback:
            self.error_callback(error_message)

    def run_sequence(self, sequence):
        raise NotImplementedError("Subclasses must implement run_sequence method")

    def abort(self):
        self.abort_requested = True

    def check_abort(self):
        if self.abort_requested:
            self.abort_requested = False
            return True
        return False

    def execute_command(self, command, *args, **kwargs):
        result = command(*args, **kwargs)
        if self.check_abort():
            raise AbortRequested("Operation aborted by user")
        return result

class AbortRequested(Exception):
    pass