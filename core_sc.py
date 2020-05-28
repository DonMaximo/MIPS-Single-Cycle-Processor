from hardware import Memory, RegisterFile, Register, MUX_2_1, ALU_32, AND_2
import utilities 
from signals import Signals

class Core_SC: 
    def __init__(self):
        self.I_Mem = Memory()
        self.D_Mem = Memory()
        self.RF = RegisterFile()
        self.RegPC = Register()
        self.signals = Signals()
        self.cycle_num = 0
        self.mode = 0

    def set_PC(self, pc):
        self.RegPC.set_data(pc)
        self.RegPC.set_write(1)

    def set_mode(self, mode):
        self.mode = mode

    def run(self, n_cycles):
        i_cycles = 0
        ending_PC = self.I_Mem.get_ending_address() 

        self.I_Mem.set_memread(1)
        self.I_Mem.set_memwrite(0)

        while (n_cycles == 0 or i_cycles < n_cycles):
            i_cycles += 1
            self.cycle_num += 1
            if ((self.mode & 2) == 0): utilities.print_new_cycle(self.cycle_num)

    
            # clock changes
            self.RegPC.clock()
            self.RF.clock()

            # read PC
            self.signals.PC = self.RegPC.read()
            self.signals.PC_4 = self.signals.PC_new = self.signals.PC + 4
            if ((self.mode & 2) == 0): utilities.println_int("PC", self.signals.PC)
            if (self.signals.PC > ending_PC): 
                if ((self.mode & 2) == 0): print("No More Instructions")
                i_cycles -= 1 
                break

            self.I_Mem.set_address(self.signals.PC)
            self.I_Mem.run()
            self.signals.instruction = self.I_Mem.get_data() 

            if ((self.mode & 2) == 0): utilities.println_int("instruction", self.signals.instruction)


            # Now have PC and Instruction

            self.signals_from_instruction(self.signals.instruction, self.signals)

            # call main_control
            self.main_control(self.signals.opcode, self.signals)
            
            # call sign_extend
            self.signals.Sign_extended_immediate = self.sign_extend(self.signals.immediate)
            
            # Write_register. Also an example of using MUX
            self.signals.Write_register = MUX_2_1(self.signals.rt, self.signals.rd, self.signals.RegDst) 
            
            # ALU control
            self.signals.ALU_operation = self.ALU_control(self.signals.ALUOp, self.signals.funct)
            
            # Calculate branch address 
            self.signals.Branch_address = self.calculate_branch_address(self.signals.PC_4, self.signals.Sign_extended_immediate)
            self.signals.Jump_address = self.calculate_jump_address(self.signals.PC_4, self.signals.instruction)
            
            # Print out signals generated in Phase 1.
            if ((self.mode & 4) == 0): utilities.print_signals_1(self.signals)

            # If phase 1 only, continue to the next instruction.
            if ((self.mode & 1) != 0):
                self.RegPC.set_data(self.signals.PC_4)  
        
            ####Executing phase 2 Functions
            self.read_RF()
            self.set_regwrite()
            self.run_ALU()
            self.run_D_mem()
            self.set_reg_wb()
            self.update_pc()
        
        return i_cycles
    def signals_from_instruction (self, instruction, sig):
        """
        Extract the following signals from instruction.
            opcode, rs, rt, rd, funct, immediate
        """

    
        sig.opcode = (instruction >> 26) & 0x0000003F
        if sig.opcode == 0:
            sig.rs = (instruction >> 21) & 0x0000001F
            sig.rt = (instruction >> 16) & 0x0000001F
            sig.rd = (instruction >> 11) & 0x0000001F
            sig.funct = instruction & 0x0000003F
            sig.immediate=0x0
        elif sig.opcode == 35 or sig.opcode == 43 or sig.opcode == 8 or sig.opcode == 4:
            sig.rs = (instruction >> 21) & 0x0000001F
            sig.rt = (instruction >> 16) & 0x0000001F
            sig.immediate = instruction & 0x0000FFFF
            sig.funct=0x0
            sig.rd=0x0

        elif sig.opcode ==2:
            sig.rs = sig.rt = sig.rd = sig.immediate = sig.funct =0

    def main_control(self, opcode, sig):
        """
        Check the type of input instruction
        """
        #set defaults for control signals 
        sig.RegDst = sig.Jump = sig.Branch = sig.MemRead = sig.MemtoReg = sig.ALUOp = sig.MemWrite = sig.ALUSrc = sig.RegWrite = 0

        #determine control signals
        if opcode == 0:             # R-Type 000000
            sig.RegWrite = 1
            sig.RegDst = 1
            sig.ALUOp = 2

        #lw
        elif opcode == 35:
            sig.ALUSrc = 1
            sig.MemtoReg = 1
            sig.RegWrite = 1
            sig.MemRead = 1
            sig.RegDst=sig.Jump=sig.Branch=sig.ALUOp=sig.MemWrite=0
        #addi
        elif opcode == 8:
            sig.ALUSrc = 1
            sig.RegWrite = 1
            sig.ALUOp = 0
            sig.RegDst = sig.Jump = sig.Branch = sig.MemRead = sig.MemtoReg = sig.MemWrite =  0
        #sw
        elif opcode == 43:
            sig.ALUSrc = 1
            sig.MemWrite = 1
            sig.RegDst=sig.Jump=sig.Branch=sig.MemRead=sig.MemtoReg=sig.ALUOp=sig.RegWrite = 0
        #j
        elif opcode == 2:
            sig.Jump =1
            sig.RegDst =  sig.Branch = sig.MemRead = sig.MemtoReg = sig.ALUOp = sig.MemWrite = sig.ALUSrc = sig.RegWrite = 0

        #beq
        elif opcode == 4:
            sig.Branch = 1
            sig.ALUOp = 1
            sig.RegDst = sig.Jump = sig.MemRead = sig.MemtoReg = sig.MemWrite = sig.ALUSrc = sig.RegWrite=0

        
        else:
            raise ValueError("Unknown opcode 0x%02X" % opcode)
        return 

    def ALU_control(self, alu_op, funct):  
        """
        Get alu_control from func field of instruction
        Input: function field of instruction
        Output: alu_control_out
       
        """
        alu_control_out = 0

        if alu_op == 0:           
            alu_control_out = 2     
        elif alu_op == 1:
            alu_control_out = 6
        elif alu_op == 2:
            if funct == 32:
                alu_control_out = 2
            elif funct == 34:
                alu_control_out = 6
            elif funct == 36:
                alu_control_out = 0
            elif funct == 37:
                alu_control_out = 1
            elif funct == 42:
                alu_control_out = 7
        else:
            raise ValueError("Unknown opcode code 0x%02X" % alu_op)
        return alu_control_out

    def sign_extend(self, immd):
        """
        Sign extend module. 
        Convert 16-bit to an int.
        Extract the lower 16 bits. 
        If bit 15 of immd is 1, compute the correct negative value (immd - 0x10000).
        """
        if (immd & 0x8000):
            immd = (immd - 0x10000) 
        else:
            immd = immd & 0x0000FFFF

        return immd

    def calculate_branch_address(self, pc_4, extended):
        addr = pc_4 + (extended << 2)
        return addr

    def calculate_jump_address(self, pc_4, instruction):
        lower_28 = (instruction & 0x03FFFFFF) << 2 
        upper_4 = (pc_4 & 0xF0000000) << 2
        addr = upper_4 | lower_28
   
        return addr
    
    
  
    def read_RF(self):
        self.RF.set_read_registers(self.signals.rs,self.signals.rt)
        self.signals.RF_read_data_1 = self.RF.get_read_data_1()
        self.signals.RF_read_data_2 = self.RF.get_read_data_2()
    
    def set_regwrite(self): 
        if (self.signals.RegWrite):
            self.RF.set_regwrite(1)
            self.signals.Write_register= MUX_2_1(self.signals.rt,self.signals.rd,self.signals.RegDst)
            self.RF.set_write_register(self.signals.Write_register)
            

    def run_ALU(self):
        self.signals.ALU_input_2 = MUX_2_1(self.signals.RF_read_data_2,self.signals.Sign_extended_immediate,self.signals.ALUSrc)
        self.signals.ALU_returned_value= ALU_32(self.signals.RF_read_data_1,self.signals.ALU_input_2,self.signals.ALU_operation)
        self.signals.ALU_result = self.signals.ALU_returned_value[0]
        self.signals.Zero = self.signals.ALU_returned_value[1]

    def run_D_mem(self):
        if (self.signals.MemWrite):
            self.D_Mem.set_memwrite(1)
            self.D_Mem.set_memread(0)
            self.D_Mem.set_data(self.signals.RF_read_data_2)
            self.D_Mem.run()
        elif(self.signals.MemRead):
            self.D_Mem.set_memread(1)
            self.D_Mem.set_memwrite(0)
            self.D_Mem.run()
            self.signals.MEM_read_data = self.D_Mem.get_data()
            
    def set_reg_wb(self):
        if (self.signals.RegWrite):
            self.RF.set_write_data(MUX_2_1(self.signals.ALU_result,self.signals.MEM_read_data,self.signals.MemtoReg))
        


    def update_pc(self):
        self.signals.PCSrc = AND_2(self.signals.Branch,self.signals.Zero)
        mux1_output = MUX_2_1(self.signals.PC_4,self.signals.Branch_address, self.signals.PCSrc)
        self.signals.PC_new = MUX_2_1(mux1_output,self.signals.Jump_address,self.signals.Jump)
        self.set_PC(self.signals.PC_new)
