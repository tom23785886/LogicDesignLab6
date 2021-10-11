`define dash 4'b1010
`define INITIAL 3'b000
`define DISPLAY 3'b001
`define GUESS 3'b010
`define RANGE 3'b011
`define SHOWANS 3'b100
module lab06(output wire [6:0]DISPLAY,output wire [3:0]DIGIT,output reg[15:0]LED,inout wire PS2_DATA,inout wire PS2_CLK,input wire rst
            ,input wire clk,input wire start,input wire cheat);
        wire clk13;
        wire clk25;
        wire clk16;
        
        reg [7:0]goal;
        reg [7:0]nextgoal;
        wire [7:0]tmpgoal;
        reg [1:0]state;
        reg [1:0]nextstate;
        reg [7:0]upperbd;
        reg [7:0]nextupbd;
        reg [7:0]lowerbd;
        reg [7:0]nextlwbd;
        reg [7:0]F;
        wire [3:0]tmp1;
        wire [3:0]tmp2;
        wire [6:0]tnum;
        
        reg [15:0]nextnums;
        reg [7:0]lownum;
        reg [7:0]nextlownum;
        reg [7:0]highnum;
        reg [7:0]nexthighnum;
        reg [24:0]cnt;
        reg [24:0]nextcnt;
        reg [15:0]nextled;

        wire [15:0] nums;
        reg [3:0] key_num;
        reg [9:0] last_key;
        
        wire [511:0] key_down;
        wire [8:0] last_change;
        wire been_ready;
        wire start_pul;
        wire clk_;

        clk_divider #(13) clk1(.clk(clk),.clk_div(clk13));
        clk_divider #(16) clk2(.clk(clk),.clk_div(clk16));
        clk_divider #(25) clk3(.clk(clk),.clk_div(clk25));
        
        initial F=8'b00000001;
        always@(posedge clk,posedge rst)
            begin
                if(rst) begin F=8'b00000001; end
                else begin F={F[6:0],F[6]^F[0]}; end
            end
        assign tnum=F%97+1;
        assign tmp1=tnum/10;
        assign tmp2=tnum%10;
        assign tmpgoal={tmp1,tmp2};      
         OnePulse op2 (
		.signal_single_pulse(start_pul),
		.signal(start),
		.clock(clk16));


       // parameter [8:0] LEFT_SHIFT_CODES  = 9'b0_0001_0010;
	//parameter [8:0] RIGHT_SHIFT_CODES = 9'b0_0101_1001;
	parameter [8:0] KEY_CODES [0:21] = {
		9'b0_0100_0101,	// 0 => 45
		9'b0_0001_0110,	// 1 => 16
		9'b0_0001_1110,	// 2 => 1E
		9'b0_0010_0110,	// 3 => 26
		9'b0_0010_0101,	// 4 => 25
		9'b0_0010_1110,	// 5 => 2E
		9'b0_0011_0110,	// 6 => 36
		9'b0_0011_1101,	// 7 => 3D
		9'b0_0011_1110,	// 8 => 3E
		9'b0_0100_0110,	// 9 => 46
		
		9'b0_0111_0000, // right_0 => 70
		9'b0_0110_1001, // right_1 => 69
		9'b0_0111_0010, // right_2 => 72
		9'b0_0111_1010, // right_3 => 7A
		9'b0_0110_1011, // right_4 => 6B
		9'b0_0111_0011, // right_5 => 73
		9'b0_0111_0100, // right_6 => 74
		9'b0_0110_1100, // right_7 => 6C
		9'b0_0111_0101, // right_8 => 75
		9'b0_0111_1101,  // right_9 => 7D
        9'b1_0101_1010,  //left_enter
        9'b0_0101_1010   //right_enter
	};
	
	//assign shift_down = (key_down[LEFT_SHIFT_CODES] == 1'b1 || key_down[RIGHT_SHIFT_CODES] == 1'b1) ? 1'b1 : 1'b0;
	
		
	KeyboardDecoder key_de (
		.key_down(key_down),
		.last_change(last_change),
		.key_valid(been_ready),
		.PS2_DATA(PS2_DATA),
		.PS2_CLK(PS2_CLK),
		.rst(rst),
		.clk(clk)
	);
    always@(*)
        begin
            if(rst) begin nextstate=`INITIAL; nextlownum=8'b1010_1010; nexthighnum=8'b1010_1010; nextupbd=8'b1001_1001; nextlwbd=8'b0000_0000;
                    nextgoal=goal; nextcnt=0; nextled=16'b0000_0000_0000_0000; end
            else
                begin
                    nextgoal=goal;
                    case(state)
                        `INITIAL:
                            begin  
                                nextcnt=0;
                                nextgoal=tmpgoal;
                                nextled=16'b0000_0000_0000_0000;
                                if(start) 
                                    begin 
                                        nextstate=`DISPLAY; 
                                        nextlownum=lownum; 
                                        nexthighnum=highnum; 
                                        nextupbd=8'b1001_1001; 
                                        nextlwbd=8'b0000_0000;
                                    end

                                else 
                                    begin 
                                        nextstate=`INITIAL; 
                                        nextlownum=8'b1010_1010; 
                                        nexthighnum=8'b1010_1010; 
                                        nextupbd=8'b1001_1001; 
                                        nextlwbd=8'b0000_0000;
                                    end
                            end
                        `DISPLAY:
                            begin 
                             if(lowerbd==upperbd)
                                begin
                                    if(cnt==25'b10000_0000_0000_0000_0000_0000) begin nextstate=`INITIAL; nextcnt=0;nextled=16'b0000_0000_0000_0000;end
                                    else begin nextstate=`DISPLAY; nextcnt=cnt+1; nextled=16'b1111_1111_1111_1111;end 
                                    nextlownum=lowerbd;
                                    nexthighnum=upperbd;
                                    nextlwbd=lowerbd;
                                    nextupbd=upperbd;
                                    /*nextstate=`DISPLAY; 
                                    nextcnt=cnt+1;*/
                                end
                            else if (been_ready && key_down[last_change] == 1'b1)   
                                begin
                                nextcnt=cnt;
                                nextled=LED;
                                    if (key_num != 4'b1111 && key_num!=4'b1100 && key_num!=4'b1101)
                                        begin
                                            nextstate=`GUESS;
                                            nextlownum=8'b1011_1011;
                                            nexthighnum={4'b1011,key_num};
                                            nextupbd=upperbd;
                                            nextlwbd=lowerbd;
                                        end
                                        
                                    else
                                        begin
                                            nextstate=`DISPLAY;
                                            nextlownum=lowerbd;
                                            nexthighnum=upperbd;
                                            nextlwbd=lowerbd;
                                            nextupbd=upperbd;
                                        end
                                end
                            else 
                                begin
                                    nextcnt=cnt;
                                    nextled=LED;
                                    nextstate=`DISPLAY; 
                                    nextlownum=lowerbd; 
                                    nexthighnum=upperbd; 
                                    nextlwbd=lowerbd;
                                    nextupbd=upperbd;
                                end
                               
                            end
                        `RANGE:
                            begin
                                nextcnt=cnt;
                                nextled=LED;
                                if(highnum>lowerbd&&highnum<upperbd)
                                    begin
                                        if(highnum>goal) 
                                            begin 
                                                nextstate=`DISPLAY;
                                                nextlownum=lowerbd; 
                                                nexthighnum=highnum; 
                                                nextlwbd=lowerbd; 
                                                nextupbd=highnum; 
                                            end
                                        else if(highnum<goal)
                                            begin
                                                nextstate=`DISPLAY;
                                                nextlownum=highnum;
                                                nexthighnum=upperbd;
                                                nextlwbd=highnum;
                                                nextupbd=upperbd;
                                            end
                                        else
                                            begin
                                                nextstate=`DISPLAY;
                                                nextlownum=highnum;
                                                nexthighnum=highnum;
                                                nextlwbd=highnum;
                                                nextupbd=highnum;
                                            end
                                    end
                                else
                                    begin
                                        nextstate=`DISPLAY;
                                        nextlownum=lowerbd;
                                        nexthighnum=upperbd;
                                        nextlwbd=lowerbd;
                                        nextupbd=upperbd;
                                    end
                            end
                        `GUESS:
                            begin
                                nextcnt=cnt;
                                nextled=LED;
                                if (been_ready && key_down[last_change] == 1'b1)   
                                begin
                                    if (key_num != 4'b1111)
                                        begin
                                            if(key_num==4'b1100||key_num==4'b1101)
                                                begin
                                                    nextstate=`RANGE;
                                                    nextlownum=lownum;
                                                    nexthighnum=highnum; 
                                                    nextupbd=upperbd;
                                                    nextlwbd=lowerbd;
                                                end
                                            else 
                                                begin
                                                    nextstate=`GUESS;
                                                    nextlownum=8'b1011_1011;
                                                    nexthighnum={highnum[3:0],key_num};
                                                    nextupbd=upperbd;
                                                    nextlwbd=lowerbd;
                                                end
                                        end
                                    else
                                        begin
                                            nextstate=`GUESS;
                                            nextlownum=8'b1011_1011;
                                            nexthighnum=highnum;
                                            nextupbd=upperbd;
                                            nextlwbd=lowerbd;
                                        end
                                end
                                else 
                                    begin 
                                        nextlownum=lownum; 
                                        nexthighnum=highnum; 
                                        nextstate=`GUESS; 
                                        nextupbd=upperbd; 
                                        nextlwbd=lowerbd; 
                                    end
                            end
                        default: begin nextstate=state; nextlownum=lownum; nexthighnum=highnum; nextupbd=upperbd; nextlwbd=lowerbd; nextcnt=cnt;
                                        nextled=LED; end
                    endcase
                end
            
        end
	always @ (posedge clk, posedge rst) begin
		if (rst) 
            begin
                lownum <= 8'b1010_1010;
                highnum <= 8'b1010_1010;
                state<=`INITIAL;
                upperbd<=8'b1001_1001;
                lowerbd<=8'b1001_1001;
                goal<=goal;
                cnt<=24'b0000_0000_0000_0000_0000_0000;
                LED<=16'b0000_0000_0000_0000;
            end 
        else 
            begin
                lownum<=nextlownum;
                highnum<=nexthighnum;
                state<=nextstate;
                upperbd<=nextupbd;
                lowerbd<=nextlwbd;
                goal<=nextgoal;
                cnt<=nextcnt;
                LED<=nextled;
		    end
	end
    assign nums=(state!=`INITIAL&&cheat==1'b1)?
            {goal,8'b1011_1011}:
            {lownum,highnum};
    /*if (been_ready && key_down[last_change] == 1'b1) begin
				if (key_num != 4'b1111)begin
					if (shift_down == 1'b1) begin
						nextnums <= {key_num, nums[15:4]};
					end else begin
						nextnums <= {nums[11:0], key_num};
					end
				end
			end*/
	SevenSegment seven_seg (
		.display(DISPLAY),
		.digit(DIGIT),
		.nums(nums),
		.rst(rst),
		.clk(clk),
        .state(state)
	);
	always @ (*) begin
		case (last_change)
			KEY_CODES[00] : key_num = 4'b0000;
			KEY_CODES[01] : key_num = 4'b0001;
			KEY_CODES[02] : key_num = 4'b0010;
			KEY_CODES[03] : key_num = 4'b0011;
			KEY_CODES[04] : key_num = 4'b0100;
			KEY_CODES[05] : key_num = 4'b0101;
			KEY_CODES[06] : key_num = 4'b0110;
			KEY_CODES[07] : key_num = 4'b0111;
			KEY_CODES[08] : key_num = 4'b1000;
			KEY_CODES[09] : key_num = 4'b1001;

			KEY_CODES[10] : key_num = 4'b0000;
			KEY_CODES[11] : key_num = 4'b0001;
			KEY_CODES[12] : key_num = 4'b0010;
			KEY_CODES[13] : key_num = 4'b0011;
			KEY_CODES[14] : key_num = 4'b0100;
			KEY_CODES[15] : key_num = 4'b0101;
			KEY_CODES[16] : key_num = 4'b0110;
			KEY_CODES[17] : key_num = 4'b0111;
			KEY_CODES[18] : key_num = 4'b1000;
			KEY_CODES[19] : key_num = 4'b1001;
            KEY_CODES[20] : key_num = 4'b1100;
            KEY_CODES[21] : key_num = 4'b1101;
			default		  : key_num = 4'b1111;
		endcase
	end

endmodule

module clk_divider(clk,clk_div);
    parameter n=4;
    input clk;
    output clk_div;
    reg [n-1:0]num;
    wire [n-1:0]nextnum;
    always@(posedge clk)
        begin
            num=nextnum;
        end
    assign nextnum=num+1;
    assign clk_div=num[n-1];
endmodule

module SevenSegment(
	output reg [6:0] display,
	output reg [3:0] digit,
	input wire [15:0] nums,
	input wire rst,
	input wire clk,
    input wire state
    );
    wire clk13;
    reg [3:0]val;
    clk_divider#(13) clk5(.clk(clk),.clk_div(clk13));
    always@(*)
    begin
        case(val)
            4'b0000:begin display=7'b1000000; end
            4'b0001:begin display=7'b1111001; end
            4'b0010:begin display=7'b0100100; end
            4'b0011:begin display=7'b0110000; end
            4'b0100:begin display=7'b0011001; end
            4'b0101:begin display=7'b0010010; end
            4'b0110:begin display=7'b0000010; end
            4'b0111:begin display=7'b1111000; end
            4'b1000:begin display=7'b0000000; end
            4'b1001:begin display=7'b0010000; end
            4'b1010:begin display=7'b0111111; end
            default: begin display=7'b1111111; end
        endcase
    end


 always@(posedge clk13)
            begin
                case(digit)
                    4'b1110: begin val=nums[7:4]; digit=4'b1101; end
                    4'b1101: begin val=nums[11:8]; digit=4'b1011; end
                    4'b1011: begin val=nums[15:12]; digit=4'b0111; end
                    4'b0111: begin val=nums[3:0]; digit=4'b1110; end
                    default: begin val=nums[3:0]; digit=4'b1110; end 
                endcase
            end
    
endmodule


module KeyboardDecoder(
	output reg [511:0] key_down,
	output wire [8:0] last_change,
	output reg key_valid,
	inout wire PS2_DATA,
	inout wire PS2_CLK,
	input wire rst,
	input wire clk
    );
    
    parameter [1:0] INIT			= 2'b00;
    parameter [1:0] WAIT_FOR_SIGNAL = 2'b01;
    parameter [1:0] GET_SIGNAL_DOWN = 2'b10;
    parameter [1:0] WAIT_RELEASE    = 2'b11;
    
	parameter [7:0] IS_INIT			= 8'hAA;
    parameter [7:0] IS_EXTEND		= 8'hE0;
    parameter [7:0] IS_BREAK		= 8'hF0;
    
    reg [9:0] key;		// key = {been_extend, been_break, key_in}
    reg [1:0] state;
    reg been_ready, been_extend, been_break;
    
    wire [7:0] key_in;
    wire is_extend;
    wire is_break;
    wire valid;
    wire err;
    
    wire [511:0] key_decode = 1 << last_change;
    assign last_change = {key[9], key[7:0]};
    
    KeyboardCtrl_0 inst (
		.key_in(key_in),
		.is_extend(is_extend),
		.is_break(is_break),
		.valid(valid),
		.err(err),
		.PS2_DATA(PS2_DATA),
		.PS2_CLK(PS2_CLK),
		.rst(rst),
		.clk(clk)
	);
	
	OnePulse op (
		.signal_single_pulse(pulse_been_ready),
		.signal(been_ready),
		.clock(clk)
	);
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		state <= INIT;
    		been_ready  <= 1'b0;
    		been_extend <= 1'b0;
    		been_break  <= 1'b0;
    		key <= 10'b0_0_0000_0000;
    	end else begin
    		state <= state;
			been_ready  <= been_ready;
			been_extend <= (is_extend) ? 1'b1 : been_extend;
			been_break  <= (is_break ) ? 1'b1 : been_break;
			key <= key;
    		case (state)
    			INIT : begin
    					if (key_in == IS_INIT) begin
    						state <= WAIT_FOR_SIGNAL;
    						been_ready  <= 1'b0;
							been_extend <= 1'b0;
							been_break  <= 1'b0;
							key <= 10'b0_0_0000_0000;
    					end else begin
    						state <= INIT;
    					end
    				end
    			WAIT_FOR_SIGNAL : begin
    					if (valid == 0) begin
    						state <= WAIT_FOR_SIGNAL;
    						been_ready <= 1'b0;
    					end else begin
    						state <= GET_SIGNAL_DOWN;
    					end
    				end
    			GET_SIGNAL_DOWN : begin
						state <= WAIT_RELEASE;
						key <= {been_extend, been_break, key_in};
						been_ready  <= 1'b1;
    				end
    			WAIT_RELEASE : begin
    					if (valid == 1) begin
    						state <= WAIT_RELEASE;
    					end else begin
    						state <= WAIT_FOR_SIGNAL;
    						been_extend <= 1'b0;
    						been_break  <= 1'b0;
    					end
    				end
    			default : begin
    					state <= INIT;
						been_ready  <= 1'b0;
						been_extend <= 1'b0;
						been_break  <= 1'b0;
						key <= 10'b0_0_0000_0000;
    				end
    		endcase
    	end
    end
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		key_valid <= 1'b0;
    		key_down <= 511'b0;
    	end else if (key_decode[last_change] && pulse_been_ready) begin
    		key_valid <= 1'b1;
    		if (key[8] == 0) begin
    			key_down <= key_down | key_decode;
    		end else begin
    			key_down <= key_down & (~key_decode);
    		end
    	end else begin
    		key_valid <= 1'b0;
			key_down <= key_down;
    	end
    end

endmodule

module OnePulse (
	output reg signal_single_pulse,
	input wire signal,
	input wire clock
	);
	reg signal_delay;

	always @(posedge clock) begin
		if (signal == 1'b1 & signal_delay == 1'b0)
		  signal_single_pulse <= 1'b1;
		else
		  signal_single_pulse <= 1'b0;

		signal_delay <= signal;
	end
endmodule

module KeyboardCtrl#(
   parameter SYSCLK_FREQUENCY_HZ = 100000000
)(
    output reg [7:0] key_in,
    output reg is_extend,
    output reg is_break,
	output reg valid,
    output err,
    inout PS2_DATA,
    inout PS2_CLK,
    input rst,
    input clk
);
//////////////////////////////////////////////////////////
// This Keyboard  Controller do not support lock LED control
//////////////////////////////////////////////////////////

    parameter RESET          = 3'd0;
	parameter SEND_CMD       = 3'd1;
	parameter WAIT_ACK       = 3'd2;
    parameter WAIT_KEYIN     = 3'd3;
	parameter GET_BREAK      = 3'd4;
	parameter GET_EXTEND     = 3'd5;
	parameter RESET_WAIT_BAT = 3'd6;
    
    parameter CMD_RESET           = 8'hFF; 
    parameter CMD_SET_STATUS_LEDS = 8'hED;
	parameter RSP_ACK             = 8'hFA;
	parameter RSP_BAT_PASS        = 8'hAA;
    
    parameter BREAK_CODE  = 8'hF0;
    parameter EXTEND_CODE = 8'hE0;
    parameter CAPS_LOCK   = 8'h58;
    parameter NUM_LOCK    = 8'h77;
    parameter SCR_LOCK    = 8'h7E;
    
    wire [7:0] rx_data;
	wire rx_valid;
	wire busy;
	
	reg [7:0] tx_data;
	reg tx_valid;
	reg [2:0] state;
	reg [2:0] lock_status;
	
	always @ (posedge clk, posedge rst)
	  if(rst)
	    key_in <= 0;
	  else if(rx_valid)
	    key_in <= rx_data;
	  else
	    key_in <= key_in;
	
	always @ (posedge clk, posedge rst)begin
	  if(rst)begin
	    state <= RESET;
        is_extend <= 1'b0;
        is_break <= 1'b1;
		valid <= 1'b0;
		lock_status <= 3'b0;
		tx_data <= 8'h00;
		tx_valid <= 1'b0;
	  end else begin
	    is_extend <= 1'b0;
	    is_break <= 1'b0;
	    valid <= 1'b0;
	    lock_status <= lock_status;
	    tx_data <= tx_data;
	    tx_valid <= 1'b0;
	    case(state)
	      RESET:begin
	          is_extend <= 1'b0;
              is_break <= 1'b1;
		      valid <= 1'b0;
		      lock_status <= 3'b0;
		      tx_data <= CMD_RESET;
		      tx_valid <= 1'b0;
			  state <= SEND_CMD;
	        end
		  
		  SEND_CMD:begin
		      if(busy == 1'b0)begin
			    tx_valid <= 1'b1;
				state <= WAIT_ACK;
			  end else begin
			    tx_valid <= 1'b0;
				state <= SEND_CMD;
		      end
		    end
	      
		  WAIT_ACK:begin
		      if(rx_valid == 1'b1)begin
			    if(rx_data == RSP_ACK && tx_data == CMD_RESET)begin
				  state <= RESET_WAIT_BAT;
				end else if(rx_data == RSP_ACK && tx_data == CMD_SET_STATUS_LEDS)begin
				  tx_data <= {5'b00000, lock_status};
				  state <= SEND_CMD;
				end else begin
				  state <= WAIT_KEYIN;
				end
			  end else if(err == 1'b1)begin
			    state <= RESET;
			  end else begin
			    state <= WAIT_ACK;
			  end
		    end
			
		  WAIT_KEYIN:begin
		      if(rx_valid == 1'b1 && rx_data == BREAK_CODE)begin
			    state <= GET_BREAK;
			  end else if(rx_valid == 1'b1 && rx_data == EXTEND_CODE)begin
			    state <= GET_EXTEND;
			  end else if(rx_valid == 1'b1)begin
			    state <= WAIT_KEYIN;
				valid <= 1'b1;
			  end else if(err == 1'b1)begin
			    state <= RESET;
			  end else begin
			    state <= WAIT_KEYIN;
			  end
		    end
		    
		  GET_BREAK:begin
		      is_extend <= is_extend;
		      if(rx_valid == 1'b1)begin
			    state <= WAIT_KEYIN;
                valid <= 1'b1;
				is_break <= 1'b1;
			  end else if(err == 1'b1)begin
			    state <= RESET;
			  end else begin
			    state <= GET_BREAK;
			  end
		    end
			
		  GET_EXTEND:begin
		      if(rx_valid == 1'b1 && rx_data == BREAK_CODE)begin
		        state <= GET_BREAK;
		        is_extend <= 1'b1;
		      end else if(rx_valid == 1'b1)begin
		        state <= WAIT_KEYIN;
                valid <= 1'b1;
		        is_extend <= 1'b1;
			  end else if(err == 1'b1)begin
			    state <= RESET;
		      end else begin
		        state <= GET_EXTEND;
		      end
		    end
			
		  RESET_WAIT_BAT:begin
		      if(rx_valid == 1'b1 && rx_data == RSP_BAT_PASS)begin
			    state <= WAIT_KEYIN;
			  end else if(rx_valid == 1'b1)begin
			    state <= RESET;
			  end else if(err == 1'b1)begin
			    state <= RESET;
			  end else begin
			    state <= RESET_WAIT_BAT;
			  end
		    end
		  default:begin
		      state <= RESET;
		      valid <= 1'b0;
		    end
		endcase
	  end
	end
	
    Ps2Interface #(
      .SYSCLK_FREQUENCY_HZ(SYSCLK_FREQUENCY_HZ)
    ) Ps2Interface_i(
      .ps2_clk(PS2_CLK),
      .ps2_data(PS2_DATA),
      
      .clk(clk),
      .rst(rst),
      
      .tx_data(tx_data),
      .tx_valid(tx_valid),
      
      .rx_data(rx_data),
      .rx_valid(rx_valid),
      
      .busy(busy),
      .err(err)
    );
        
endmodule
