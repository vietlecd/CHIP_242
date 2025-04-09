// module RISC_CPU (
//     input clk,
//     input reset,
//     input [31:0] instruction,
//     output reg [31:0] pc,
//     output reg [31:0] alu_result
// );

//     // Define internal registers and wires
//     reg [31:0] registers [0:31];
//     reg [31:0] alu_operand1, alu_operand2;
//     wire [4:0] rs1, rs2, rd;
//     wire [6:0] opcode;
//     wire [2:0] funct3;
//     wire [6:0] funct7;

//     // Instruction fields
//     assign opcode = instruction[6:0];
//     assign rs1 = instruction[19:15];
//     assign rs2 = instruction[24:20];
//     assign rd = instruction[11:7];
//     assign funct3 = instruction[14:12];
//     assign funct7 = instruction[31:25];

//     // ALU operations
//     always @(*) begin
//         case (opcode)
//             7'b0110011: begin // R-type instructions
//                 alu_operand1 = registers[rs1];
//                 alu_operand2 = registers[rs2];
//                 case (funct3)
//                     3'b000: alu_result = (funct7 == 7'b0000000) ? alu_operand1 + alu_operand2 : alu_operand1 - alu_operand2; // ADD/SUB
//                     3'b111: alu_result = alu_operand1 & alu_operand2; // AND
//                     3'b110: alu_result = alu_operand1 | alu_operand2; // OR
//                     3'b100: alu_result = alu_operand1 ^ alu_operand2; // XOR
//                     default: alu_result = 0;
//                 endcase
//             end
//             default: alu_result = 0;
//         endcase
//     end

//     // Program counter update
//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             pc <= 0;
//         end else begin
//             pc <= pc + 4;
//         end
//     end

//     // Register write-back
//     always @(posedge clk) begin
//         if (opcode == 7'b0110011) begin // R-type instructions
//             registers[rd] <= alu_result;
//         end
//     end

// endmodule

module Address_Mux(
    input sel,                    // Pc: 1; Toán hạng: 0
    input [4:0] add_pc_out,       // địa chỉ ra của PC (5-bit)
    input [4:0] operand_addr,     // toán hạng của lệnh (5-bit)
    output reg [4:0] selected_addr   // địa chỉ được ch�?n ( đầu ra )
);
// sử dụng parameter để dễ dàng thay đổi độ rộng
parameter WIDTH = 5;

always @(*) 
    begin
        if(sel)
            selected_addr = add_pc_out;
        else
            selected_addr = operand_addr;
    end
endmodule


// Program Counter
module PC(
    input clk,
    input reset,
    input ld_pc,
    input inc_pc,
    input halt,
    input [4:0] add_pc_in,
    output reg [4:0] add_pc_out
);

always @(posedge clk) 
    begin
        if(reset)
            add_pc_out <= 0;
        else if (ld_pc)
            add_pc_out <= add_pc_in;
        else if(inc_pc)
            add_pc_out <= add_pc_out + 1;
        else if (halt)
            begin
                add_pc_out <= add_pc_out;
                $display("stop program");
                $finish;
            end
    end
endmodule

// Memory
module Memory(
    input clk,
    input rd,
    input wr,
    input halt,
    input [4:0] selected_addr,
    input data_e,  // ***** data_e là gì????????
    input [7:0] data_in,
    output reg [7:0] data_out,
    output reg [7:0] instruction
);
    // khai báo bộ nhớ
    reg [7:0] mem [0:31]; // 32 | 8bit|

    // xử lý
    always @(posedge clk) 
        begin
            if (halt)
                begin
                    $display("stop program");
                    $finish;
                end
            if(wr && data_e) begin
                begin
                    mem[selected_addr] = data_in;
                end
            end
           if(rd)
                begin
                    data_out <= mem[selected_addr];
                end
            
           instruction <= mem[selected_addr]; // lấy lệnh từ bộ nhớ 
        end
endmodule

// Instruction Register
module IR(
    input clk,
    input reset,
    input ld_ir,
    input [7:0] instruction,
    output reg [4:0] operand_addr,
    output reg [2:0] opcode
);

always @(posedge clk) 
    begin
        if(reset) // reset = 1 => IR = 0
            begin
                operand_addr <= 0;
                opcode <= 0;
            end
        else if(ld_ir)
            begin
                operand_addr <= instruction[4:0];
                opcode <= instruction[7:5];
            end
    end
endmodule

// ALU
module ALU(
    input [2:0] opcode,
    input [7:0] data_out, // inB
    input [7:0] acc_out, // inA
    output reg [7:0] alu_out,
    output reg is_zero
);

always @(*) 
    begin
        is_zero = (acc_out == 0);
        case(opcode)
            3'b000: alu_out = acc_out;
            3'b001: alu_out = acc_out;
            3'b010: alu_out = acc_out + data_out;
            3'b011: alu_out = acc_out & data_out;
            3'b100: alu_out = acc_out ^ data_out;
            3'b101: alu_out = data_out;
            3'b110: alu_out = acc_out ;
            3'b111: alu_out = acc_out;
            default: alu_out = 0;
        endcase
    end

endmodule

// Accumulator Register
module ACC(
    input clk,
    input reset,
    input [7:0] alu_out,
    input ld_ac,
    output reg [7:0] acc_out
);

always @(posedge clk) 
    begin
        if(reset)
            acc_out <= 0;
        else if(ld_ac)
            acc_out <= alu_out;
            // giwux nguyên gtri nếu không có tín hiệu load
    end
endmodule

// Controller
module Controller(
    input clk, reset,
    input [2:0] opcode,
    input is_zero,
    input hazard_checked, 
    output reg sel, rd, wr, ld_ir, halt, inc_pc, ld_ac, ld_pc, data_e,  
    output reg stall
);

    // trạng thái của bộ đi�?u khiển
    reg [2:0] state;
    parameter [2:0] 
        INST_ADDR = 0,
        INST_FETCH = 1,
        INST_LOAD = 2,
        IDLE = 3,
        OP_ADDR = 4,
        OP_FETCH = 5,
        ALU_OP = 6,
        STORE = 7;

    //  Chuyển trạng thái
always @(posedge clk) begin
    if (reset) begin
        state <= INST_ADDR;
        sel <= 1;
        rd <= 0;
        wr <= 0;
        ld_ir <= 0;
        halt <= 0;
        inc_pc <= 0;
        ld_ac <= 0;
        ld_pc <= 0;
        data_e <= 0;
        stall <= 0; 
    end else begin
            if (hazard_checked) begin
                stall <= 1; 
            end else begin
                stall <= 0; 
            end
             
            case (state)
                    INST_ADDR: begin
                        sel = 1; // ch�?n địa chỉ của PC
                        state = INST_FETCH;
                    end
                    INST_FETCH: begin
                        sel = 1;
                        rd = 1; // đ�?c lệnh từ bộ nhớ
                        state = INST_LOAD;
                    end
                    INST_LOAD: begin
                        sel = 1;
                        rd = 1;
                        ld_ir = 1;
                        state = IDLE;
                    end
                    IDLE: begin
                        sel = 1;
                        rd = 1;
                        ld_ir = 1;
                        state = OP_ADDR;
                    end
                    OP_ADDR: begin
                        halt = (opcode == 3'b000);
                        inc_pc = 1;
                        rd = 1;
                        state = OP_FETCH;
                    end
                    OP_FETCH: begin
                        rd = (opcode == 3'b010 | opcode == 3'b011 | opcode == 3'b100 | opcode == 3'b101);
                        state = ALU_OP;
                    end
                    ALU_OP: begin
                        rd = (opcode == 3'b010 | opcode == 3'b011 | opcode == 3'b100 | opcode == 3'b101);
                        inc_pc = (opcode == 3'b001 && is_zero);
                        ld_pc = (opcode == 3'b111);
                        data_e = (opcode == 3'b110);
                        state = STORE;
                    end
                    STORE: begin
                        rd = (opcode == 3'b010 | opcode == 3'b011 | opcode == 3'b100 | opcode == 3'b101);
                        ld_ac = (opcode == 3'b010 | opcode == 3'b011 | opcode == 3'b100 | opcode == 3'b101);
                        ld_pc = (opcode == 3'b111);
                        wr = (opcode == 3'b110);
                        data_e = (opcode == 3'b110);
                        state = INST_ADDR;
                    end
                    default: state = INST_ADDR;
                endcase
    end
end
    // always @(*) 
    //     begin
    //         // sel = 0;
    //         // rd = 0;
    //         // ld_ir = 0;
    //         // halt = 0;
    //         // inc_pc = 0;
    //         // ld_ac = 0;
    //         // ld_pc = 0;
    //         // wr = 0;
    //         // data_e = 0;           
    //         case (state)
    //                 INST_ADDR: begin
    //                     sel = 1; // ch�?n địa chỉ của PC
    //                     // state = INST_FETCH;
    //                 end
    //                 INST_FETCH: begin
    //                     sel = 1;
    //                     rd = 1; // đ�?c lệnh từ bộ nhớ
    //                     // state = INST_LOAD;
    //                 end
    //                 INST_LOAD: begin
    //                     sel = 1;
    //                     rd = 1;
    //                     ld_ir = 1;
    //                     // state = IDLE;
    //                 end
    //                 IDLE: begin
    //                     sel = 1;
    //                     rd = 1;
    //                     ld_ir = 1;
    //                     // state = OP_ADDR;
    //                 end
    //                 OP_ADDR: begin
    //                     halt = (opcode == 3'b000);
    //                     inc_pc = 1;
    //                     // state = OP_FETCH;
    //                 end
    //                 OP_FETCH: begin
    //                     rd = (opcode == 3'b010 | opcode == 3'b011 | opcode == 3'b100 | opcode == 3'b101);
    //                     // state = ALU_OP;
    //                 end
    //                 ALU_OP: begin
    //                     rd = (opcode == 3'b010 | opcode == 3'b011 | opcode == 3'b100 | opcode == 3'b101);
    //                     inc_pc = (opcode == 3'b001 && is_zero);
    //                     ld_pc = (opcode == 3'b111);
    //                     data_e = (opcode == 3'b110);
    //                     // state = STORE;
    //                 end
    //                 STORE: begin
    //                     rd = (opcode == 3'b010 | opcode == 3'b011 | opcode == 3'b100 | opcode == 3'b101);
    //                     ld_ac = (opcode == 3'b010 | opcode == 3'b011 | opcode == 3'b100 | opcode == 3'b101);
    //                     ld_pc = (opcode == 3'b111);
    //                     wr = (opcode == 3'b110);
    //                     data_e = (opcode == 3'b110);
    //                     // state = INST_ADDR;
    //                 end
    //                 // default: state = INST_ADDR;
    //             endcase
    //     end

endmodule

// RISC CPU
module RISC_CPU(
    input clk,
    input reset
);

    // Tín hiệu kết nối giữa các module
    wire [4:0] add_pc_out, operand_addr, selected_addr, add_pc_in;
    wire [7:0] instruction, data_out, data_in, alu_out, acc_out;
    wire [2:0] opcode;
    wire is_zero;
    wire sel, rd, wr, ld_ir, halt, inc_pc, ld_ac, ld_pc, data_e;

    // Module Program Counter (PC)
    PC Top1 (clk, reset, ld_pc, inc_pc, halt, add_pc_in [4:0], add_pc_out [4:0]);
    Address_Mux Top2 (sel, add_pc_out[4:0], operand_addr[4:0], selected_addr [4:0]);
    Memory Top3 (clk, rd, wr,halt, selected_addr [4:0], data_e, data_in [7:0], data_out [7:0], instruction [7:0]);
    IR Top4(clk, reset, ld_ir, instruction [7:0], operand_addr [4:0], opcode [2:0]);
    ACC Top5  (clk, reset, alu_out [7:0], ld_ac, acc_out [7:0]);
    ALU Top6 (opcode [2:0], data_out [7:0], acc_out [7:0], alu_out [7:0], is_zero);
    Controller Top7 (clk, reset, opcode [2:0], is_zero, sel, rd, wr, ld_ir, halt, inc_pc, ld_ac, ld_pc, data_e);
endmodule