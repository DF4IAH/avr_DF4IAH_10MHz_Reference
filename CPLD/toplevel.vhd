------------------------------------------------------
-- DF4IAH  10 MHz Ref.-Osc. V2.x - CPLD device
------------------------------------------------------

----------------------------------------
--      Top level
----------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.ALL;

entity top_lev is
	port (
--		-- JTAG
--		TDI 		: in  std_logic;
--		TDO 		: out std_logic;
--		TCK 		: in  std_logic;
--		TMS 		: in  std_logic;


		-- GLOBAL
		RESET 		: in  std_logic;

		-- Clock Divider
		C_20MHZ 	: in  std_logic;
		C_10MHZ		: out std_logic;
		C_10KHZ 	: in  std_logic;
		C_PPS		: in  std_logic;
		C_2MHZ5		: out std_logic;
		
		-- Phase Error Determination
		PPS		: in  std_logic;
		Sync  		: in  std_logic;
		GateTrig 	: out std_logic;
		Gate  		: in  std_logic;
		Phase 		: out std_logic
	);
end;

architecture structural of top_lev is
	component clock_div  -- component declaration for clock_div
		port (
			RESET:	 in  std_logic;
			C_20MHZ: in  std_logic;
			C_10MHZ: out std_logic;
			C_2MHZ5: out std_logic
		);
	end component;

	component capture  -- component declaration for capture
		port (
			RESET:	  in  std_logic;
--			C_2MHZ5:  in  std_logic;
			C_10KHZ:  in  std_logic;
--			C_PPS:	  in  std_logic;
			Sync:	  in  std_logic;
			GateTrig: out std_logic;
			Gate: 	  in  std_logic;
			Phase:	  out std_logic
		);
	end component;

-- Declaration of top_lev signals
signal C_2MHZ5_local: std_logic;

begin  --  structural description begins
	clock_div_0: clock_div 
		port map (
			RESET => RESET,
			C_20MHZ => C_20MHZ, 
			C_10MHZ => C_10MHZ, 
			C_2MHZ5 => C_2MHZ5_local 
		);

	capture_0: capture
		port map (
			RESET => RESET,
--			C_2MHZ5 => C_2MHZ5_local,
			C_10KHZ => C_10KHZ,
--			C_PPS => C_PPS,
			Sync => Sync,
			GateTrig => GateTrig,
			Gate => Gate,
			Phase => Phase
		);

	C_2MHZ5 <= C_2MHZ5_local;
end structural;


--	-- 8< --

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.ALL;

entity clock_div is
	port (
		RESET:	 in  std_logic;
		C_20MHZ: in  std_logic;
		C_10MHZ: out std_logic;
		C_2MHZ5: out std_logic
	);
end clock_div;

architecture BEHAVIORAL of clock_div is
	signal Q: 	unsigned(2 downto 0);
	signal count:	std_logic_vector(2 downto 0);

begin 
	process (C_20MHZ, RESET)
	begin
	    if RESET = '1' then
	        Q <= "000";

	    elsif rising_edge(C_20MHZ) then
	        Q <= Q + 1;
	    end if;
	end process;

	count <= std_logic_vector(Q); -- type conversion
	C_10MHZ <= count(0);
	C_2MHZ5 <= count(2);
end BEHAVIORAL;


--	-- 8< --

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.ALL;

entity capture is
	port (
		RESET:	  in  std_logic;
--		C_2MHZ5:  in  std_logic;
		C_10KHZ:  in  std_logic;
--		C_PPS:	  in  std_logic;
		Sync:	  in  std_logic;
		GateTrig: out std_logic;
		Gate: 	  in  std_logic;
		Phase:	  out std_logic
	);
end capture;

architecture BEHAVIORAL of capture is
signal PhaseJKFF:	std_logic;

begin
	-- Monoflop activation
	process (C_10KHZ, RESET)
	begin
	    if RESET = '1' then
		GateTrig <= '0';

	    else
		GateTrig <= C_10KHZ;
	    end if;
	end process;

	-- Phase determination
	process (C_10KHZ, Sync)
	begin
	    if Sync = '0' then
		PhaseJKFF <= '0';

	    elsif rising_edge(C_10KHZ) then
		PhaseJKFF <= '1';
	    end if;
	end process;

	-- Output Enable
	process (PhaseJKFF, Gate)
	begin
	    if Gate = '1' then
		Phase <= PhaseJKFF;

	    else
		Phase <= 'Z';
	    end if;
	end process;
end BEHAVIORAL;

