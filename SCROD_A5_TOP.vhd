----------------------------------------------------------------------------------
-- Company			: IDLAB
-- Engineer			: Khanh Le
-- 
-- Create Date		: 17:30:05 11/30/2016 
-- Module Name		: SCROD_A5_TOP 
-- Project Name	: BOREHOLE MUON DETECTOR
-- Target Devices	: Spartan 6 XC6SLX150T-3FGG676
-- Tool versions  : ISE VERSION 14.7
-- Description		: Master controller for BMD, makes trigger decesions and sends data back to pc
--
-- Revision			: 1
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;
Library UNISIM;
use UNISIM.vcomponents.all;
use work.all;
use work.BMD_definitions.all;


entity SCROD_A5_TOP is
	Port ( --incoming signals 
		MASTER_CLK_P     : IN STD_LOGIC;--input clock either 127MHz form osc or remote clock form 
		MASTER_CLK_N 	  : IN STD_LOGIC;
		--incoming signals from top dc plan
		TOP_DC_SC_RX 	  : IN STD_LOGIC;
		TOP_DC_SC_DIN    : IN STD_LOGIC;
		--incoming signals from bot dc plan
		BOT_DC_SC_RX 	  : IN STD_LOGIC;
		BOT_DC_SC_DIN    : IN STD_LOGIC;
		--outgoing signals to top dc plan
		TOP_TRIG         : INOUT STD_LOGIC;
		TOP_SC_DC_TX  	  : OUT STD_LOGIC;
		TOP_SC_DC_DOUT   : OUT STD_LOGIC;
		--outgoing signals to bot dc plan
		BOT_TRIG         : INOUT STD_LOGIC;
		BOT_SC_DC_TX 	  : OUT STD_LOGIC;
		BOT_SC_DC_DOUT   : OUT STD_LOGIC;
		--outgoin clocks 
		TOP_SCROD_DC_CLK_P : OUT STD_LOGIC; 
		TOP_SCROD_DC_CLK_N : OUT STD_LOGIC;
		BOT_SCROD_DC_CLK_P : OUT STD_LOGIC; 
		BOT_SCROD_DC_CLK_N : OUT STD_LOGIC;
		SPARE_CLK_P			 : OUT STD_LOGIC;
		SPARE_CLK_N        : OUT STD_LOGIC;
		--fiber signals
		MGTTXFAULT       : IN STD_LOGIC;
		MGTMOD0          : IN STD_LOGIC;
		MGTLOS           : IN STD_LOGIC;
		MGTTXDIS         : OUT STD_LOGIC;
		MGTMOD2          : OUT STD_LOGIC;
		MGTMOD1          : OUT STD_LOGIC;
		MGTRXP           : IN STD_LOGIC;
		MGTRXN           : IN STD_LOGIC;
		MGTTXP           : OUT STD_LOGIC;
		MGTTXN           : OUT STD_LOGIC;
		MGTCLK0P   		  : IN STD_LOGIC; 
		MGTCLK0N  		  : IN STD_LOGIC; 
		MGTCLK1P         : IN STD_LOGIC;
		MGTCLK1N         : IN STD_LOGIC;
		--ethernaet signals--needs to change direction later on depending on use
		RJ45_ACK_P		  : IN  STD_LOGIC;--ACK for CDT that scrod received trigger
		RJ45_ACK_N       : IN  STD_LOGIC;
		RJ45_TRG_P       : OUT STD_LOGIC;--local trigger to CDT, ie.. moun hit on both plans
		RJ45_TRG_N       : OUT STD_LOGIC;
		RJ45_CLK_P       : IN STD_LOGIC;--remote clock for system 125MHz
		RJ45_CLK_N       : IN STD_LOGIC;
		RJ45_RSV_P       : OUT STD_LOGIC;--unused
		RJ45_RSV_N       : OUT STD_LOGIC;
		--SRAM:
		RAM_A				  : OUT STD_LOGIC_VECTOR(21 downto 0);-- RAM address line         
		RAM_IO			  : INOUT STD_LOGIC_VECTOR(7 downto 0);-- RAM IO data line     
		RAM_CE1n			  : OUT STD_LOGIC := '1';                                         
		RAM_CE2			  : OUT STD_LOGIC := '0';                           
		RAM_OEn			  : OUT STD_LOGIC := '1';                       
		RAM_WEn			  : OUT STD_LOGIC := '1';                         
		--general IO pins
		J1_079_Y6 		  : OUT STD_LOGIC;--needs to change direction later on depending on use
		J1_078_AA3 		  : OUT STD_LOGIC;
		J1_077_AA4 		  : OUT STD_LOGIC;
		J1_075_AB3 		  : OUT STD_LOGIC;
		J1_074_AB4       : OUT STD_LOGIC;
		J1_073_AB5       : OUT STD_LOGIC;
		J1_071_AC3       : OUT STD_LOGIC;
		J1_070_AC4       : OUT STD_LOGIC;
		J1_069_AD3       : OUT STD_LOGIC;
		J1_067_W7_N      : OUT STD_LOGIC;
		J1_066_W8_P      : OUT STD_LOGIC;
		J1_062_AB10_P    : OUT STD_LOGIC;
		J1_061_AB11_N    : OUT STD_LOGIC;
		J1_060_AE2_P     : OUT STD_LOGIC;
		J1_059_AE1_N     : OUT STD_LOGIC;
--		J1_056_AC2_P     : OUT STD_LOGIC;
--		J1_055_AC1_N     : OUT STD_LOGIC;
--		J1_052_AA2_P     : OUT STD_LOGIC;
--		J1_051_AA1_N     : OUT STD_LOGIC;
--		J1_048_W2_P      : IN STD_LOGIC;
--		J1_047_W1_N      : OUT STD_LOGIC;
--		J1_044_U2_P      : IN STD_LOGIC;
--		J1_043_U1_N      : OUT STD_LOGIC;
--		J1_042_R2 	     : OUT STD_LOGIC
		--Triggers from DCs
		E_TRIG           : IN  STD_LOGIC_VECTOR(3 downto 0);
		SCROD_TRIG       : OUT STD_LOGIC_VECTOR(3 downto 0);
		CDT_TRIG         : OUT STD_LOGIC);
end SCROD_A5_TOP;

architecture Behavioral of SCROD_A5_TOP is
--declearing outgoing internal signals
signal internal_data_clk      : std_logic := '0';
signal internal_fpga_clk      : std_logic := '0';


--declearing internal communication signals
signal OUTPUT_REGISTER	: GPR;
signal oops_reset 		: std_logic := '0';
signal tx_busy       	: std_logic := '0';
signal data_clk         : std_logic := '0';
signal cdt_trigger      : std_logic := '0';
signal cdt_ack          : std_logic := '0';
signal start_pc_send    : std_logic := '0';
signal start_readout 	: std_logic_vector(4 downto 0) :=(others=>'0');
signal dc_no_response   : std_logic_vector(4 downto 0) :=(others=>'0');


signal dc_fifo_rd_en   : std_logic := '0';
signal dc_fifo_dout   	: std_logic_vector(31 downto 0); 
signal dc_fifo_empty  	: std_logic;
signal start_con     	: std_logic;

signal wave_reset   		: std_logic := '0';
signal wave_write_en  	: std_logic := '0';
signal wave_clock   		: std_logic;
signal wave_data_in  	: std_logic_vector(31 downto 0);

signal top_bot_word     : std_logic;
signal readout_start 	: std_logic_vector(4 downto 0);
signal ped_readout_dcnum: std_logic_vector(3 downto 0);
signal fix_win 			: std_logic_vector(8 downto 0):=(others=>'0');	
signal asic_enable_bits : std_logic_vector(3 downto 0);
signal demux_mode 		: std_logic_vector(3 downto 0);
signal dc_rd_demux_busy : std_logic;


--attribute keep: boolean;
--attribute keep of TOP_SC_DC_DOUT: signal is true;

------------------------------------------------------------------------------------------------------------
-----------------------------converting lvds osc clock into internal clock----------------------------------
------------------------------------------------------------------------------------------------------------
begin
--CLK_FANOUT_1TO2 : entity work.CLK_FANOUT
--  port map
--   (-- Clock in ports
--    CLK_IN1_P => RJ45_CLK_P,
--    CLK_IN1_N => RJ45_CLK_N,
--    -- Clock out ports
--    CLK_OUT1 => internal_fpga_clk,--125MHz
--    CLK_OUT2 => internal_data_clk);--25MHz


CLK_FANOUT_1TO2 : entity work.CLK_FANOUT
  port map
   (-- Clock in ports
    CLK_IN1_P => MASTER_CLK_P,
    CLK_IN1_N => MASTER_CLK_N,
    -- Clock out ports
    CLK_OUT1 => internal_fpga_clk,--125MHz
    CLK_OUT2 => internal_data_clk);--25MHz
	 
	 
	 
	 
clk_sync: process(internal_fpga_clk) begin
if rising_edge(internal_fpga_clk) then
	data_clk <= internal_data_clk; 
end if;
end process;	 
	 
------------------------------------------------------------------------------------------------------------
-----------------------------converting internal clock into lvds for output to DC---------------------------
------------------------------------------------------------------------------------------------------------

--sending clock to top plane--	
top_clk_p_oddr2 : ODDR2
generic map(
	DDR_ALIGNMENT => "NONE", -- Sets output alignment to "NONE", "C0", "C1"
	INIT => '0', -- Sets initial state of the Q output to '0' or '1'
	SRTYPE => "SYNC") -- Specifies "SYNC" or "ASYNC" set/reset
port map (
	Q => TOP_SCROD_DC_CLK_P, -- 1-bit output data --internal_fpga_clk
	C0 => data_clk, -- 1-bit clock input
	C1 => not data_clk, -- 1-bit clock input
	CE => '1', -- 1-bit clock enable input
	D0 => '1', -- 1-bit data input (associated with C0)
	D1 => '0', -- 1-bit data input (associated with C1)
	R => '0', -- 1-bit reset input
	S => '0'); -- 1-bit set input

top_clk_n_oddr2 : ODDR2
generic map(
	DDR_ALIGNMENT => "NONE", -- Sets output alignment to "NONE", "C0", "C1"
	INIT => '0', -- Sets initial state of the Q output to '0' or '1'
	SRTYPE => "SYNC") -- Specifies "SYNC" or "ASYNC" set/reset
port map (
	Q => TOP_SCROD_DC_CLK_N, -- 1-bit output data
	C0 => not data_clk, -- 1-bit clock input
	C1 => data_clk, -- 1-bit clock input
	CE => '1', -- 1-bit clock enable input
	D0 => '1', -- 1-bit data input (associated with C0)
	D1 => '0', -- 1-bit data input (associated with C1)
	R => '0', -- 1-bit reset input
	S => '0'); -- 1-bit set input

				  
--sending clock to bottom plane--
bot_clk_p_oddr2 : ODDR2
generic map(
	DDR_ALIGNMENT => "NONE", -- Sets output alignment to "NONE", "C0", "C1"
	INIT => '0', -- Sets initial state of the Q output to '0' or '1'
	SRTYPE => "SYNC") -- Specifies "SYNC" or "ASYNC" set/reset
port map (
	Q => BOT_SCROD_DC_CLK_P, -- 1-bit output data
	C0 => data_clk, -- 1-bit clock input
	C1 => not data_clk, -- 1-bit clock input
	CE => '1', -- 1-bit clock enable input
	D0 => '1', -- 1-bit data input (associated with C0)
	D1 => '0', -- 1-bit data input (associated with C1)
	R => '0', -- 1-bit reset input
	S => '0'); -- 1-bit set input

bot_clk_n_oddr2 : ODDR2
generic map(
	DDR_ALIGNMENT => "NONE", -- Sets output alignment to "NONE", "C0", "C1"
	INIT => '0', -- Sets initial state of the Q output to '0' or '1'
	SRTYPE => "SYNC") -- Specifies "SYNC" or "ASYNC" set/reset
port map (
	Q => BOT_SCROD_DC_CLK_N, -- 1-bit output data
	C0 => not data_clk, -- 1-bit clock input
	C1 => data_clk, -- 1-bit clock input
	CE => '1', -- 1-bit clock enable input
	D0 => '1', -- 1-bit data input (associated with C0)
	D1 => '0', -- 1-bit data input (associated with C1)
	R => '0', -- 1-bit reset input
	S => '0'); -- 1-bit set input
				  
------------------------------------------------------------------------------------------------------------
-----------------------------communication from SCROD to DC-------------------------------------------------
------------------------------------------------------------------------------------------------------------
oops_reset       <= OUTPUT_REGISTER(0)(0);--master reset
top_bot_word     <= OUTPUT_REGISTER(1)(0);--lets SCROD knows if its the 1:top or 0:bot
asic_enable_bits <= OUTPUT_REGISTER(7)(3 downto 0);

comm_process : entity work.SCROD_DC_COMM
PORT MAP (CLK             	=> internal_fpga_clk,--125MHz clock
			 DATA_CLK        	=> data_clk,         --20MHz clock
			 PC_SEND         	=> start_pc_send,    --signal to start sending data to pc 
			 START_READOUT    => start_readout,    --starts reading out dc
			 DC_NO_GO         => dc_no_response,   --current dc readout timed out send dead beef 
			 TOP_BOT          => top_bot_word,     --lets pc know if SCROD is top ot bot comes from register 1 bit 0
			 OOPS_RESET       => oops_reset, 	   --reset all modules to idle comes from register 0 bit 0				   
			 --incoming signals from top plan
          TOP_DC_SC_RX 		=> TOP_DC_SC_RX,     --receiving trigger from top DC plan
          TOP_DC_SC_DIN   	=> TOP_DC_SC_DIN,    --data from top DC plan
			 --incoming signals from bot plan
			 BOT_DC_SC_RX 		=> BOT_DC_SC_RX,		--receiving trigger from bot DC plan
			 BOT_DC_SC_DIN   	=> BOT_DC_SC_DIN,    --data from bot DC plan
			 --outgoing signals to top plan
			 TOP_SC_DC_TX  	=> TOP_SC_DC_TX,     --sending trigger from scrod to top plan
			 TOP_SC_DC_DOUT  	=> TOP_SC_DC_DOUT,   --data from scrod to top DC plan
			 --outgoing signals to bot plan
			 BOT_SC_DC_TX 		=> BOT_SC_DC_TX,     --sending trigger from scrod to bot plan
			 BOT_SC_DC_DOUT  	=> BOT_SC_DC_DOUT,   --data from scrod to bot DC plan
			 --internal control register
			 TX_BUSY         	=> tx_busy,  			--internal busy signal 
			 CTIME            => open,
			 OUTPUT_REGISTER 	=> OUTPUT_REGISTER,
			 --trigger signal 
			 DC_TRIG          => E_TRIG,           --trigger from all DC
			 ASIC_EN_BITS     => asic_enable_bits, --comes from output register 3 bits 3-0
			 SCROD_TRIG       => SCROD_TRIG,       --trigger to all DC from scrod
			 CDT_TRIG         => cdt_trigger,      --trigger to CDT for readout
			 TOP_TRIG         => TOP_TRIG,
			 BOT_TRIG         => BOT_TRIG,			 
			 --data from top and bot plans
			 DC_FIFO_RD_EN		=> dc_fifo_rd_en,
			 DC_FIFO_DOUT		=> dc_fifo_dout,
			 DC_FIFO_EMPTY		=> dc_fifo_empty,
			 START_CON        => start_con,
			 DONE_CON         => open,
			 --data to wave fifo for pc 
			 WAVE_FIFO_CLK    => wave_clock,
			 WAVE_FIFO_RST		=> wave_reset,
			 WAVE_WR_EN			=> wave_write_en,
			 WAVE_DIN			=> wave_data_in,
			 --fiber signals		
			 mgttxfault 		=> MGTTXFAULT,
			 mgtmod0 			=> MGTMOD0,
			 mgtlos 				=> MGTLOS,
			 mgttxdis 			=> MGTTXDIS,
			 mgtmod2 			=> MGTMOD2,
			 mgtmod1 			=> MGTMOD1,
			 mgtrxp 				=> MGTRXP,
			 mgtrxn 				=> MGTRXN,
			 mgttxp 				=> MGTTXP,
			 mgttxn 				=> MGTTXN,
			 mgtclk1p 			=> MGTCLK1P,
			 mgtclk1n 			=> MGTCLK1N);




------------------------------------------------------------------------------------------------------------
-----------------------------communication from SCROD to CDTC-----------------------------------------------
------------------------------------------------------------------------------------------------------------
--outputing trigger through RJ-45 to CDT
RJ45_TRG_P <= cdt_trigger;
RJ45_TRG_N <= not cdt_trigger;
CDT_TRIG   <= cdt_trigger;

				  		
--CDT_ACK_OFBUFD : IBUFDS
--	generic map (
--		DIFF_TERM    => FALSE, -- Differential Termination
--		IBUF_LOW_PWR => TRUE, -- Low power (TRUE) vs. performance (FALSE) setting for referenced I/O standards
--		IOSTANDARD   => "DEFAULT")
--	port map (
--		O  => cdt_ack, -- Buffer output
--		I  => RJ45_ACK_P, -- Diff_p buffer input (connect directly to top-level port)
--		IB => RJ45_ACK_N); -- Diff_n buffer input (connect directly to top-level port)

------------------------------------------------------------------------------------------------------------
-----------------------------loading dc wave data into internal BRAM----------------------------------------
------------------------------------------------------------------------------------------------------------
readout_start    		<= OUTPUT_REGISTER(5)(4 downto 0);  --readout signal from pc
ped_readout_dcnum    <= OUTPUT_REGISTER(5)(15 downto 12);--dc number of ped data saved in sram, use fix win as start window in this mode
demux_mode           <= OUTPUT_REGISTER(8)(3 downto 0);  --0:send raw data, 1:send pedsub data, 2:send ped data from sram, 3: save ped data to sram
fix_win              <= OUTPUT_REGISTER(12)(8 downto 0); -- bit 15: '1'=> use fixed start win and (8 downto 0) is the fixed start win

DC_READOUT_DEMUX: entity work.DataDemuxControl 
	PORT MAP(
		--GENERAL I/O
		CLK    		=> internal_fpga_clk, --125MHz clock
		OOPS   		=> oops_reset, 		 --reset all modules to idle comes from register 0 bit 0
		DC_RD_BUSY  => dc_rd_demux_busy,  --internal busy signal
		RD_SRT 		=> readout_start, 	 --comes from output register 7 bit 4-0
		DC_PED_RD   => ped_readout_dcnum, --comes from output register 7 bit 15-12
		MODE 			=> demux_mode, 		 --comes from output register 8 bit 3-0
		FIX_WIN 		=> fix_win,           --comes from output register 12 bit 8-0
		
		--I/O TO communication process		
		TX_BUSY 		=> tx_busy, 
		SRT_CON     => start_con,
		SRT_PC_SEND => start_pc_send,
		START_RD		=> start_readout,
		NO_GO 		=> dc_no_response,

		WAVE_WR_EN	=> wave_write_en,
		WAVE_DIN 	=> wave_data_in,
		WAVE_CLK 	=> wave_clock,
		WAVE_RST    => wave_reset,
		DC_FIFO_EMPTY   => dc_fifo_empty,
		DC_FIFO_RD_EN   => dc_fifo_rd_en,
		DC_FIFO_DOUT    => dc_fifo_dout);


end Behavioral;


