from experiment_operations import ExperimentOperations, AbortRequested
from time import sleep

class OpenChamberOperations(ExperimentOperations):
    def __init__(self, config, syringe_pump, selector_valves, disc_pump):
        self.config = config
        self.sp = syringe_pump
        self.sv = selector_valves
        self.dp = disc_pump
        self.chamber_volume = config['chamber_volume_ul']
        super().__init__(self.config, self.sp, self.sv)

    def run_sequence(self, sequence):
        print(sequence)
        try:
            sequence_name = sequence['sequence_name']
            port = int(sequence['fluidic_port'])
            flow_rate = int(sequence['flow_rate'])
            volume = int(sequence['volume'])
            incubation_time = int(sequence['incubation_time'])
            fill_tubing_with = sequence['fill_tubing_with']

            self.sp.reset_chain()

            if sequence_name == "Add Reagent" or sequence_name == "Add Buffer":
                self.add_reagent(port, flow_rate, volume, incubation_time, fill_tubing_with)
            elif sequence_name == "Clear and Add Reagent":
                self.clear_and_add_reagent(port, flow_rate, volume, incubation_time, fill_tubing_with)
            elif sequence_name == "Wash with Constant Flow":
                self.wash_with_constant_flow(port, flow_rate, volume, fill_tubing_with)
            else:
                self.report_error(f"Unknown sequence: {sequence_name}")
        except AbortRequested:
            self.report_error("Operation aborted by user")

    def incubate(self, time_minutes, progress_prefix="Incubating"):
        total_time = time_minutes * 60  # Convert minutes to seconds
        for i in range(total_time):
            sleep(1)
            if i % 5 == 0:  # Check abort every 5 seconds during incubation
                self.execute_command(lambda: None)  # Dummy command to check abort
            progress = (i + 1) / total_time * 100
            self.update_progress(f"{progress_prefix}: {i+1}/{total_time} seconds", progress)

    def clear_and_add_reagent(self, port, flow_rate, volume, incubation_time, fill_tubing_with_port):
        # Clear previous liquid in tubings by 1) dispensing sv_to_sp into waste and 2) dispensing sp_to_oc into sample and aspirate
        # Then add reagent from {port}
        self.update_progress(f"Clearing tubings and adding reagent: port={port}, flow_rate={flow_rate}, volume={volume}")
        speed_code = self.sp.flow_rate_to_speed_code(flow_rate)
        volume = min(self.config['chamber_volume_ul'], volume)
        try:
            self.sv.open_port(port)
            # Clear previous buffer in tubings (sv_to_sp)
            #self.sp.reset_chain()
            self.sp.extract(2, self.config['tubing_fluid_amount_sv_to_sp_ul'], self.config['syringe_pump']['speed_code_limit'])
            self.sp.dispense_to_waste(self.config['syringe_pump']['speed_code_limit'])
            # Assume reagent volume is greater than 'tubing_fluid_amount_sv_to_sp_ul'
            self.sp.extract(2, volume - self.config['tubing_fluid_amount_sv_to_sp_ul'], self.config['syringe_pump']['speed_code_limit'])
            self.execute_command(self.sp.execute)
            if fill_tubing_with_port:
                self.sv.open_port(int(fill_tubing_with_port))
            # Clear previous buffer in tubings (sp to chamber)
            self.sp.extract(2, self.config['tubing_fluid_amount_sv_to_sp_ul'], self.config['syringe_pump']['speed_code_limit'])
            self.execute_command(self.sp.execute)
            self.dp.aspirate(10)
            # Assume reagent volume is greater than 'tubing_fluid_amount_sp_to_oc_ul'
            self.sp.dispense(3, self.config['tubing_fluid_amount_sp_to_oc_ul'], speed_code)
            self.execute_command(self.sp.execute)
            self.dp.aspirate(10)
            # Push reagent to open chamber
            self.sp.dispense(3, volume - self.config['tubing_fluid_amount_sp_to_oc_ul'], speed_code)
            self.sp.extract(2, self.config['tubing_fluid_amount_sp_to_oc_ul'], self.config['syringe_pump']['speed_code_limit'])
            self.sp.dispense(3, self.config['tubing_fluid_amount_sp_to_oc_ul'], speed_code)
            self.execute_command(self.sp.execute)
            self.incubate(incubation_time, "Incubating with reagent/buffer")
        except:
            self.report_error(f"Error : {str(e)}")

    def add_reagent(self, port, flow_rate, volume, incubation_time, fill_tubing_with_port):
        # Clear previous liquid in tubings by 1) dispensing sv_to_sp into waste and 2) dispensing sp_to_oc into sample and aspirate
        # Then add reagent from {port}
        self.update_progress(f"Adding reagent/buffer: port={port}, flow_rate={flow_rate}, volume={volume}")
        speed_code = self.sp.flow_rate_to_speed_code(flow_rate)
        volume = min(self.config['chamber_volume_ul'], volume)
        try:
            # Assume syringe_volume > (sp_to_oc + sv_to_sp) > chamber_volume > sp_to_oc > sv_to_sp > overflow (sp_to_oc + sv_to_sp - chamber_volume)
            if fill_tubing_with_port:
                self.sv.open_port(int(fill_tubing_with_port))
            else:
                self.sv.open_port(port)
            # Draw sv_to_sp into syringe
            #self.sp.reset_chain()
            self.sp.extract(2, self.config['tubing_fluid_amount_sv_to_sp_ul'], self.config['syringe_pump']['speed_code_limit'])
            # Discard overflow amount
            overflow = self.config['tubing_fluid_amount_sp_to_oc_ul'] + self.config['tubing_fluid_amount_sv_to_sp_ul'] - volume
            self.sp.dispense(3, overflow, speed_code)
            self.dp.aspirate(10)
            self.execute_command(self.sp.execute)
            # Push reagent to sample
            self.sp.dispense(3, volume - self.config['tubing_fluid_amount_sp_to_oc_ul'], speed_code)
            self.sp.extract(2, self.config['tubing_fluid_amount_sp_to_oc_ul'], self.config['syringe_pump']['speed_code_limit'])
            self.sp.dispense(3, self.config['tubing_fluid_amount_sp_to_oc_ul'], speed_code)
            self.dp.aspirate(10)
            self.execute_command(self.sp.execute)
            self.incubate(incubation_time, "Incubating with reagent/buffer")
        except:
            self.report_error(f"Error : {str(e)}")

    def wash_with_constant_flow(self, port, flow_rate, volume, fill_tubing_with_port):
        self.update_progress(f"Washing with constant flow: port={port}, flow_rate={flow_rate}, volume={volume}")
        speed_code = self.sp.flow_rate_to_speed_code(flow_rate)
        volume = min(self.config['syringe_pump']['volume_ul'], volume)
        try:
            self.sv.open_port(port)
            # No need to clear previous liquid in tubings (sv_to_sp)
            #self.sp.reset_chain()
            self.sp.extract(2, volume - self.config['tubing_fluid_amount_sv_to_sp_ul'], self.config['syringe_pump']['speed_code_limit'])
            self.execute_command(self.sp.execute, True)
            if fill_tubing_with_port:
                self.sv.open_port(fill_tubing_with_port)
            self.sp.extract(2, self.config['tubing_fluid_amount_sv_to_sp_ul'], self.config['syringe_pump']['speed_code_limit'])
            self.sp.dispense(3, volume, speed_code)
            # Push reagent to open chamber
            self.dp.start(0.3)
            self.execute_command(self.sp.execute, True)
            self.dp.stop()
            if fill_tubing_with_port:
                # Wash with additional amount of buffer in tubing sp_to_oc and fill with next reagent
                self.sp.extract(2, self.config['tubing_fluid_amount_sp_to_oc_ul'], self.config['syringe_pump']['speed_code_limit'])
                self.sp.dispense(3, self.config['tubing_fluid_amount_sp_to_oc_ul'], speed_code)
                self.dp.start(0.3)
                self.execute_command(self.sp.execute, True)
                self.dp.stop()
            sleep(1)
        except:
            self.report_error(f"Error : {str(e)}")