`define		zero_word		32'd0

`define		lui				7'b0110111
`define		auipc			7'b0010111
`define		jal				7'b1101111
`define		jalr			7'b1100111
`define		B_type			7'b1100011
`define		load			7'b0000011
`define		store			7'b0100011
`define		I_type			7'b0010011
`define		R_type			7'b0110011
/*
`define 	ADD  			4'b0001
`define 	SUB  			4'b0011
`define 	SLL  			4'b1100
`define 	SLT  			4'b1001
`define 	SLTU 			4'b1000
`define 	XOR  			4'b0110
`define 	SRL  			4'b1101
`define 	SRA  			4'b1110
`define 	OR   			4'b0101
`define 	AND  			4'b0100
*/
`define 	ADD  			4'd0    // 4'b0000
`define 	SUB  			4'd1    // 4'b0001

`define 	SLTU  			4'd10    // 4'b1010
`define 	SLT  			4'd11    // 4'b1011

`define 	EQCH 			4'd4    // 4'b0100

`define 	SLL 			4'd5    // 4'b0101
`define 	SRL  			4'd6    // 4'b0110
`define 	SRA  			4'd7    // 4'b0111

`define 	AND    			4'd12   // 4'b1100
`define 	OR 			    4'd13   // 4'b1101
`define     XOR  			4'd14   // 4'b1110

`define     PC_rst         32'h8000_0000
