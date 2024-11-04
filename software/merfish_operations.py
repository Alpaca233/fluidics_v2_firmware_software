from experiment_operations import ExperimentOperations, AbortRequested
from time import sleep

class MERFISHOperations(ExperimentOperations):
    def __init__(self, config, syringe_pump, selector_valves):
        super().__init__(config, syringe_pump, selector_valves)

    def run_sequence(self, sequence):
        print(sequence)
        try:
            sequence_name = sequence['sequence_name']
            port = int(sequence['fluidic_port'])
            flow_rate = int(sequence['flow_rate'])
            volume = int(sequence['volume'])
            incubation_time = int(sequence['incubation_time'])
            fill_tubing_with = sequence['fill_tubing_with']

            if sequence_name == "Flow Bleaching Buffer":
                self.flow_bleaching_buffer(port, flow_rate, volume)
            elif sequence_name == "Hybridize":
                self.hybridize(port, flow_rate, volume, incubation_time)
            elif sequence_name == "Flow Wash Buffer":
                self.flow_wash_buffer(port, flow_rate, volume)
            elif sequence_name == "Flow Imaging Buffer":
                self.flow_imaging_buffer(port, flow_rate, volume)
            else:
                self.report_error(f"Unknown sequence: {sequence_name}")
        except AbortRequested:
            self.report_error("Operation aborted by user")

    def incubate(self, time_minutes, progress_prefix="Incubating"):
        total_time = time_minutes * 60  # Convert minutes to seconds
        for i in range(total_time):
            sleep(1)
            if i % 10 == 0:  # Check abort every 10 seconds during incubation
                self.execute_command(lambda: None)  # Dummy command to check abort
            progress = (i + 1) / total_time * 100
            self.update_progress(f"{progress_prefix}: {i+1}/{total_time} seconds", progress)


    def flow_bleaching_buffer(self, port, flow_rate, volume, incubation_time, fill_tubing_with_port):
        self.update_progress(f"Running Flow Bleaching Buffer: port={port}, flow_rate={flow_rate}, volume={volume}")
        speed_code = utils.flow_rate_to_speed_code(flow_rate, self.sp)
        try:
            self.execute_command(self.sv.open_port, port)
            self.execute_command(self.sp.extract, 1, volume, speed_code)
            self.incubate(incubation_time, "Bleaching")
            if fill_tubing_with_port is not None:
                self.execute_command(self.sv.open_port, int(fill_tubing_with_port))
                self.execute_command(self.sp.extract, 1, self.sv.get_tubing_fluid_amount(fill_tubing_with_port))
            
            self.update_progress("Flow Bleaching Buffer complete", 100)
        except:
            self.report_error(f"Error in Flow Bleaching Buffer: {str(e)}")

    def hybridize(self, port, flow_rate, volume, incubation_time, fill_tubing_with_port):
        self.update_progress(f"Running Hybridize: port={port}, flow_rate={flow_rate}, volume={volume}, incubation_time={incubation_time}")
        speed_code = self.sp.flow_rate_to_speed_code(flow_rate)
        print(speed_code)
        try:
            self.execute_command(self.sp.dispense, 3, volume, 10)
            self.execute_command(self.sv, port)
            self.execute_command(self.sp.extract, 1, volume, speed_code)
            self.incubate(incubation_time, "Hybridizing")
            if fill_tubing_with_port is not None:
                self.execute_command(self.sv.open_port, int(fill_tubing_with_port))
                self.execute_command(self.sp.extract, 1, self.sv.get_tubing_fluid_amount(fill_tubing_with_port))

            self.update_progress("Hybridize complete", 100)
        except SyringePumpError as e:
            self.report_error(f"Error in Hybridize: {str(e)}")

    def flow_wash_buffer(self, port, flow_rate, volume):
        pass

    def flow_imaging_buffer(self, port, flow_rate, volume):
        pass
