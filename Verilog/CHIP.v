// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg    [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // Todo: any combinational/sequential circuit
    wire [6:0] opcode;
    wire [2:0] func3;
    wire [6:0] func7;
    wire [11:0] imm12, imm_beq, imm_sw;
    wire [19:0] imm20, imm_jal;

    // assign PC
    assign mem_addr_I = PC;

    // split the instruction into some parts that can be used
    assign opcode = mem_rdata_I[6:0];
    assign func3 = mem_rdata_I[14:12];
    assign func7 = mem_rdata_I[31:25];
    assign rd = mem_rdata_I[11:7];
    assign rs1 = mem_rdata_I[19:15];
    assign rs2 = mem_rdata_I[24:20];
    assign imm12 = mem_rdata_I[31:20];
    assign imm_beq = {mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8]};
    assign imm_sw = {mem_rdata_I[31:25], mem_rdata_I[11:7]};
    assign imm20 = mem_rdata_I[31:12];
    assign imm_jal = {mem_rdata_I[31], mem_rdata_I[19:12], mem_rdata_I[20], mem_rdata_I[30:21]};
    
    // for alu
    reg alu_running, alu_running_nxt;
    wire mulDiv_valid;
    wire mulDiv_ready;
    wire [1:0] mulDiv_mode;
    assign mulDiv_mode = 2'd0; // set the mode to be mul
    wire [63:0] mulDiv_out;
    // ALU : mulDiv
    mulDiv alu(
        .clk(clk),
        .rst_n(rst_n),
        .valid(mulDiv_valid),
        .ready(mulDiv_ready),
        .mode(mulDiv_mode),  // need only mul mode
        .in_A(rs1_data),
        .in_B(rs2_data),
        .out(mulDiv_out)
    );

    // opcode parameter
    parameter auipc = 7'b0010111;
    parameter jal = 7'b1101111;
    parameter jalr = 7'b1100111;
    parameter beq = 7'b1100011;
    parameter lw = 7'b0000011;
    parameter sw = 7'b0100011;
    parameter i_type = 7'b0010011; // addi, slti
    parameter arith = 7'b0110011;  // add, sub, xor, mul
    // parameter for func3
    parameter addi = 3'b000;
    parameter slti = 3'b010;
    parameter xor3 = 3'b100;
    // parameter for func7
    parameter add = 7'b0000000; // equals to xor7
    parameter sub = 7'b0100000;
    parameter mul = 7'b0000001;

    // assign value to regWrite
    assign regWrite = !(opcode == beq)
                      && !(opcode == sw)
                      && !(opcode == mul && alu_running == 1'b0) 
                      && !(opcode == mul && alu_running == 1'b1 && mulDiv_ready == 1'b0);
    // assign value to mem_wen_D
    assign mem_wen_D = (opcode == sw);
    // assign value to mem_addr_D
    assign mem_addr_D = (opcode == lw) ? (rs1_data + {{20{imm12[11]}}, imm12}) :
                        (opcode == sw) ? (rs1_data + {{20{imm_sw[11]}}, imm_sw}) : 32'd0;
    // assign value to mem_wdata_D
    assign mem_wdata_D = (opcode == sw) ? rs2_data : 32'd0;
    // assign value for mulDiv_valid
    assign mulDiv_valid = (opcode == arith) && (func7 == mul) && (alu_running == 0);
    // assign value to rd_data
    assign rd_data  = (opcode == auipc) ? (PC + {imm20, {12'd0}}) :
                      (opcode == jal || opcode == jalr) ? (PC + 4) :
                      (opcode == lw) ? mem_rdata_D :
                      (opcode == i_type) ? ( (func3 == addi) ? (rs1_data + {{20{imm12[11]}}, imm12}) : (rs1_data < imm12) ) :
                      (opcode == arith) ? ( (func3 == xor3) ? (rs1_data ^ rs2_data) : ((func7 == add) ? (rs1_data + rs2_data) :
                      ((func7 == sub) ? (rs1_data - rs2_data) : (mulDiv_ready == 1) ? mulDiv_out[31:0] : 32'd0)) ) : 32'd0;
    
    // combinational always block for PC
    always @(*) begin
        case(opcode)
            jal : PC_nxt = PC + {{11{imm_jal[19]}}, imm_jal, 1'd0};
            jalr: PC_nxt = rs1_data + imm12;
            beq : PC_nxt = (rs1_data == rs2_data) ? (PC + {{19{imm_beq[11]}}, imm_beq, 1'd0}) : (PC + 4);
            arith : begin
                case(func7)
                    mul: PC_nxt = (mulDiv_ready == 1) ? (PC + 4) : PC;
                    default : PC_nxt = PC + 4;
                endcase
            end
            default : PC_nxt = PC + 4;
        endcase
    end
    
    // combinational always block for alu_running
    always @(*) begin
        case(opcode)
            arith : begin
                case(func7)
                    mul : begin
                        case(alu_running)
                            1'b0 : alu_running_nxt = 1'b1;
                            1'b1 : begin
                                case(mulDiv_ready)
                                    1'b0 : alu_running_nxt = 1'b1;
                                    1'b1 : alu_running_nxt = 1'b0;
                                endcase
                            end 
                        endcase
                    end
                    default: alu_running_nxt = 1'b0;
                endcase
            end
            default : alu_running_nxt = 1'b0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            alu_running <= 1'b0;
        end
        else begin
            PC <= PC_nxt;
            alu_running <= alu_running_nxt;
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end
endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2
    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: and, 3: avg
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter AND = 3'd3;
    parameter AVG = 3'd4;
    parameter OUT  = 3'd5;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign ready = (state == OUT) ? 1 : 0;
    assign out = (state == OUT) ? shreg : 0;
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid == 0) begin
                    state_nxt = IDLE;
                end
                else begin
                    case(mode)
                        2'd0 : state_nxt = MUL;
                        2'd1 : state_nxt = DIV;
                        2'd2 : state_nxt = AND;
                        2'd3 : state_nxt = AVG;
                    endcase
                end
            end
            MUL : begin
                state_nxt = (counter != 31) ? MUL : OUT;
            end
            DIV : begin
                state_nxt = (counter != 31) ? DIV : OUT;
            end
            AND : state_nxt = OUT;
            AVG : state_nxt = OUT;
            OUT : state_nxt = IDLE;
            default : state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(*) begin
        case(state)
            MUL : counter_nxt = counter + 1;
            DIV : counter_nxt = counter + 1;
            default : counter_nxt = 5'd0;
        endcase
    end
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MUL : alu_out = shreg[0] ? alu_in : 0;
            DIV : alu_out = (shreg[62:31] >= alu_in) ? {1'b0, shreg[62:31] - alu_in} : {1'b0, shreg[62:31]};
            AND : alu_out = {1'b0, shreg[31:0] & alu_in};
            AVG : alu_out = (shreg[32:0] + alu_in) >> 1;
            default : alu_out = 0;
        endcase
    end
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE: shreg_nxt = valid ? {32'd0, in_A} : 0;
            MUL : shreg_nxt = {1'b0, shreg[63:1]} + {alu_out[31:0], 31'd0};
            DIV : shreg_nxt = (shreg[62:31] >= alu_in) ? {alu_out[31:0], shreg[30:0], 1'b1} : {alu_out[31:0], shreg[30:0], 1'b0};
            AND : shreg_nxt = {32'd0, alu_out[31:0]};
            AVG : shreg_nxt = {32'd0, alu_out[31:0]};
            OUT : shreg_nxt = 0;
            default: shreg_nxt = shreg;
        endcase
    end
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            counter <= 5'd0;
            alu_in <= 32'd0;
            shreg <= 64'd0;
        end
        else begin
            state <= state_nxt;
            counter <= counter_nxt;
            alu_in <= alu_in_nxt;
            shreg <= shreg_nxt;
        end
    end
endmodule