-- Required libraries.

-- Localised functions.

-- Localised values.

--- Class creation.
local Maths = {}

--- Initiates a new Maths object.
-- @return The new Maths.
function Maths.new( options )

	-- Create ourselves
	local self = {}

    function self.sign( n )
		return (n > 0 and 1) or (n == 0 and 0) or -1
    end

    function self.clamp( n, min, max )
        return math.min( math.max( n, min ), max )
    end

    function self.pmod( n, m )
        return ( n % m + m ) % m
    end

    -- Return the Maths object
	return self

end

return Maths
