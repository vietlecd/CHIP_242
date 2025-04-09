// module Address_Mux_tb;
//     // khai báo các tín hiệu
//     reg sel;
//     reg [4:0] add_pc_out, operand_addr;
//     wire [4:0] selected_addr;

//     // Khởi tạo module Address_Mux
//     Address_Mux uut (
//         .sel(sel),
//         .add_pc_out(add_pc_out),
//         .operand_addr(operand_addr),
//         .selected_addr(selected_addr)
//     );

//     // Test cases
//     initial 
//         begin
//             // Test 1 - AM: sel = 1 => chọn PC
//             sel = 1;
//             add_pc_out = 5'b00001;
//             operand_addr = 5'b11111;
//             #10;
//             $display("TEST 1 _ AM: sel = 1 | PC = 00001 | operand_addr = 11111 | selected_addr = %b", selected_addr);
        
//             // $finish;
//         end
// endmodule

// // Testbench PC
// module PC_tb;
//     // khai báo các tín hiệu
//     reg clk, reset, ld_pc, inc_pc, halt;
//     reg [4:0] add_pc_in;
//     wire [4:0] add_pc_out;
    
//     // khởi tạo module PC
//     PC uut (
//         .clk(clk),
//         .reset(reset),
//         .ld_pc(ld_pc),
//         .inc_pc(inc_pc),
//         .halt(halt),
//         .add_pc_in(add_pc_in),
//         .add_pc_out(add_pc_out)
//     );

//     // Tạo tín hiệu clock
//     initial begin
//         clk = 0;
//         forever #5 clk = ~clk; // Đảo trạng thái clk mỗi 5 đơn vị thời gian
//     end

//     // Test cases
//     initial 
//         begin
//             // Test 1 - PC: reset = 1 => PC = 0
//             reset = 1;
//             ld_pc = 0;
//             inc_pc = 0;
//             halt = 0;
//             add_pc_in = 5'b00111;
//             #10;
//             $display("TEST 1 - PC: reset = 1 | add_pc_out = %b", add_pc_out);

//             // Test 2 - PC: khởi tạo
//             // reset = 1; // kích hoạt reset để khởi tạo PC
//             // ld_pc = 0;
//             // inc_pc = 0;
//             // add_pc_in = 5'b00111;
//             // #10;
//             // tắt reset sau khi khởi tạo
//             reset = 0;
//             ld_pc = 1;
//             add_pc_in = 5'b00111;
//             #10;
//             $display("TEST 2 - PC: add_pc_out = %b", add_pc_out);
//         end
// endmodule

// // Testbench Memory
// module Memory_tb;
//     // khai báo các tín hiệu
//     reg clk, rd, wr, halt, data_e;
//     reg [4:0] selected_addr;
//     reg [7:0] data_in;
//     wire [7:0] data_out, instruction;

//     // Khởi tạo module Memory
//     Memory uut (
//         .clk(clk),
//         .rd(rd),
//         .wr(wr),
//         .halt(halt),
//         .selected_addr(selected_addr),
//         .data_e(data_e),
//         .data_in(data_in),
//         .data_out(data_out),
//         .instruction(instruction)
//     );

//     // Tạo tín hiệu clock
//     initial 
//         begin
//             clk =0;
//             forever #5 clk = ~clk;
//         end

//     // Test cases
//     initial 
//         begin
//         // Test 1 - Mem
//         // clk =1 ;
//         wr = 1;
//         rd = 0;
//         halt = 0;
//         selected_addr = 5'b00011;
//         data_in = 8'b01011000; // Ghi giá trị 01011000 vào địa chỉ 3
//         #10;
//         $display("TEST 1 - Mem: wr = 1 | rd = 0 | mem[selected_addr] = %b | data_out = %b | instruction = %b", data_in, data_out, instruction);
//         // Đọc dữ liệu từ bộ nhớ
//         wr = 0;
//         rd = 1;
//         #10;
//         $display("TEST 2 - Mem: wr = 0 | rd = 1 | mem[selected_addr] = %b | data_out = %b | instruction = %b", data_in, data_out, instruction);
//         end
// endmodule

// // Testbench Instruction register
// module IR_tb;
//     // Khai báo các tín hiệu
//     reg clk, reset, ld_ir;
//     reg [7:0] instruction;
//     wire [4:0] operand_addr;
//     wire [2:0] opcode;

//     // Khởi tạo module IR
//     IR uut (
//         .clk(clk),
//         .reset(reset),
//         .ld_ir(ld_ir),
//         .instruction(instruction),
//         .operand_addr(operand_addr),
//         .opcode(opcode)
//     );

//     // Tạo tín hiệu clock
//     initial
//         begin
//             clk = 0;
//             forever #5 clk = ~clk;
//         end

//     //Test cases
//     initial 
//         begin
//             // Test 1 - IR
//             reset = 1;
//             ld_ir = 0;
//             instruction = 8'b01011000;
//             #10;
//             $display("TEST 1 - IR: reset = 1 | operand_addr = %b | opcode = %b", operand_addr, opcode);
        
//             // Test 2 - IR
//             reset =0;
//             ld_ir =1;
//             instruction = 8'b01011000;
//             #10;
//             $display("TEST 2 - IR: ld_ir = 1 |operand_addr = %b | opcode = %b", operand_addr, opcode);

//         end
// endmodule

// // Testbench ALU
// module ALU_tb;
//     reg [7:0] data_out, acc_out;
//     reg [2:0] opcode;
//     wire [7:0] alu_out;
//     wire is_zero;

//     // Khởi tạo module ALU
//     ALU uut (
//         .data_out(data_out),
//         .acc_out(acc_out),
//         .opcode(opcode),
//         .alu_out(alu_out),
//         .is_zero(is_zero)
//     );

//     // Tạo tín hiệu clock
//     // initial 
//     //     begin
//     //         clk = 0;
//     //         forever #5 clk = ~clk;
//     //     end

//     // Test cases
//     initial 
//         begin
//             // Test 1 - ALU
//             acc_out = 8'b00001111; // 15
//             data_out = 8'b00000001; // 1
//             opcode = 3'b000; // HLT
//             #10;
//             $display("TEST 1 - ALU: opcode = 000 | acc_out = %b | data_out = %b | alu_out = %b | is_zero = %b", acc_out, data_out, alu_out, is_zero);

//             // Test 2 - ALU
//             acc_out = 8'b00001111; // 15
//             data_out = 8'b00000001; // 1
//             opcode = 3'b100; // SKZ
//             #10;
//             $display("TEST 2 - ALU: opcode = 100 | acc_out = %b | data_out = %b | alu_out = %b | is_zero = %b", acc_out, data_out, alu_out, is_zero);
        
        

//         end

// endmodule

// // Testbench ACC
// module ACC_tb;
//     reg clk, reset, ld_ac;
//     reg [7:0] alu_out;
//     wire [7:0] acc_out;

//     // Khởi tạo module ACC
//     ACC uut (
//         .clk(clk),
//         .reset(reset),
//         .ld_ac(ld_ac),
//         .alu_out(alu_out),
//         .acc_out(acc_out)
//     );

//     // Tạo tín hiệu clock
//     initial 
//         begin
//             clk = 0;
//             forever #5 clk = ~clk;
//         end

//     // Test cases
//     initial 
//         begin
//             // Test 1 - ACC
//             reset = 1;
//             ld_ac = 0;
//             alu_out = 8'b00001111; // 15
//             #10;
//             $display("TEST 1 - ACC: reset = 1 |ld_ac = 0 | alu_out = %b | acc_out = %b", alu_out, acc_out);

//             // Test 2 - ACC
//             reset = 0;
//             ld_ac = 1;
//             alu_out = 8'b00001111; // 15
//             #10;
//             $display("TEST 2 - ACC: reset = 0 |ld_ac = 1 | alu_out = %b | acc_out = %b", alu_out, acc_out);

//         end
// endmodule


// // Testbench Controller
// module Controller_tb;
//     reg clk, reset;
//     reg [2:0] opcode;
//     reg is_zero;
//     wire sel, rd, wr, ld_ir, halt, inc_pc, ld_ac, ld_pc, data_e;

//     Controller uut (
//         .clk(clk),
//         .reset(reset),
//         .opcode(opcode),
//         .is_zero(is_zero),
//         .sel(sel),
//         .rd(rd),
//         .wr(wr),
//         .ld_ir(ld_ir),
//         .ld_ac(ld_ac),
//         .halt(halt),
//         .inc_pc(inc_pc),
//         .ld_pc(ld_pc),
//         .data_e(data_e)
//     );

//     // Tạo tín hiệu clock
//     initial 
//         begin
//             clk = 0;
//             forever #5 clk = ~clk;         
//         end

//     // Test cases
//     initial 
//         begin
//             // Test 1 - Controller
//             reset = 0;
//             #10;
//             reset = 1;
//             opcode = 3'b000;
//             is_zero = 0;
//             #10;
//             $display("TEST 1 - Controller: reset = %b | opcode = %b | is_zero = %b | sel = %b | rd = %b | wr = %b | ld_ir = %b | ld_ac = %b | halt = %b | inc_pc = %b | ld_pc = %b | data_e = %b", 
//                                             reset, opcode, is_zero, sel, rd, wr, ld_ir, ld_ac, halt, inc_pc, ld_pc, data_e);
        
//             //Kết thúc mô phỏng
//             $finish;

//         end

// endmodule

// Testbench RISC_CPU

// module RISC_CPU_TB;
//     reg clk, reset;
//     RISC_CPU uut (
//         .clk(clk),
//         .reset(reset)
//     );

//     initial begin
//         clk = 0;
//         forever #5 clk = ~clk;
//     end

// //     initial begin
// //     // Khởi tạo bộ nhớ với các lệnh
// //     uut.Top3.mem[0] = 8'b01011000; // Lệnh 1
// //     uut.Top3.mem[1] = 8'b01100100; // Lệnh 2
// //     uut.Top3.mem[2] = 8'b00000000; // HLT
// // end

//     // initial begin
//     //     reset = 1; #10;
//     //     reset = 0;
//     //     // Load test instructions into memory
//     //     // Example: LDA 0x1F, ADD 0x20, STO 0x21, HLT
//     //     $display("Time: %0t | Reset: %b | PC: %b | Instruction: %b | Opcode: %b | Accumulator: %b | Data Out: %b | Halt: %b", 
//     //              $time, reset, uut.ProgramCounter.add_pc_out, uut.MEM.instruction, uut.InstructionRegister.opcode, uut.Accumulator.acc_out, uut.MEM.data_out, uut.CTRL.halt);

//     //     #100;
//     //           $display("Time: %0t | Reset: %b | PC: %b | Instruction: %b | Opcode: %b | Accumulator: %b | Data Out: %b | Halt: %b", 
//     //              $time, reset, uut.ProgramCounter.add_pc_out, uut.MEM.instruction, uut.InstructionRegister.opcode, uut.Accumulator.acc_out, uut.MEM.data_out, uut.CTRL.halt);

//     //     $finish;
//     // end

//     initial begin
//     reset = 1; 
//     uut.Top7.ld_ir = 0;
//     // uut.Top3.rd = 0;
//     uut.Top3.mem[0] = 8'b01011000;
//     #10;
//     $display("Instruction: %b | Opcode: %b | operand_addr: %b", uut.Top3.instruction, uut.Top4.opcode, uut.Top4.operand_addr);
    
//     // Kích hoạt đọc lệnh từ bộ nhớ
//     reset = 0;
//     uut.Top7.ld_ir = 1;
//     // uut.Top3.mem[0] = 8'b01011000;
//     #10;

//     // Kiểm tra giá trị của instruction và opcode
//     $display("Instruction: %b | Opcode: %b | operand_addr: %b", uut.Top3.instruction, uut.Top4.opcode, uut.Top4.operand_addr);

//     // Kết thúc mô phỏng
//     $finish;
// end

// endmodule

module RISC_CPU_TB;
    reg clk, reset;
    RISC_CPU uut (
        .clk(clk),
        .reset(reset)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Tạo xung nhịp chu kỳ 10 đơn vị
    end

    initial begin
        reset = 1;           // Kích hoạt reset
        // Khởi tạo bộ nhớ bằng hệ thống
        $readmemb("init.mem", uut.Top3.mem); 
        $monitor("Time: %0t | Reset: %b | PC: %b | inc_pc: %b | Instruction: %b | Opcode: %b | inA: %b | inB: %b | alu_out: %b | is_zero: %b | Accumulator: %b | Data Out: %b | Halt: %b", 
             $time, reset, uut.Top1.add_pc_out, uut.Top7.inc_pc, uut.Top3.instruction, uut.Top4.opcode, uut.Top6.acc_out, uut.Top6.data_out, uut.Top6.acc_out, uut.Top6.is_zero, uut.Top5.acc_out, uut.Top3.data_out, uut.Top7.halt);

        #10;                 // Đợi reset hoàn tất
        reset = 0;           // Tắt reset
        #500;                 // Đợi CPU xử lý lệnh
        // Hiển thị kết quả
        // $display("Instruction: %b | Opcode: %b | operand_addr: %b", 
        //     uut.Top3.instruction, uut.Top4.opcode, uut.Top4.operand_addr);

        //  $display("Time: %0t | Reset: %b | PC: %b | Instruction: %b | Opcode: %b | Accumulator: %b | Data Out: %b | Halt: %b", 
        //          $time, reset, uut.Top1.add_pc_out, uut.Top3.instruction, uut.Top4.opcode, uut.Top5.acc_out, uut.Top3.data_out, uut.Top7.halt);

        // #50;
        // Hiển thị kết quả cuối cùng
        // $display("Final State:");
        $finish;
    end

    initial begin
    $dumpfile("waveform.vcd"); // Tên tệp VCD
    $dumpvars(0, RISC_CPU_TB); // Ghi tất cả các tín hiệu trong module testbench
end 

endmodule