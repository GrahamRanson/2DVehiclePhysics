-- Required libraries.

-- Localised functions.
local sqrt = math.sqrt
local atan2 = math.atan2
local cos = math.cos
local sin = math.sin

-- Localised values.

--- Class creation.
local Vector2 = {}

function Vector2.len( x, y )
    return sqrt( x * x + y * y )
end

function Vector2.angle( x, y )
    return atan2( y, x )
end

--- Initiates a new Vector2 object.
-- @return The new Vector2.
function Vector2.new( x, y )

	-- Create ourselves
	local self = {}

    self.x = x or 0
    self.y = y or 0

    self.set = function( x, y )
    	self.x = x
        self.y = y
    end

    self.copy = function( v )
    	self.x = v.x
        self.y = v.y
    	return self
    end

    self.len = function()
    	return sqrt( self.x * self.x + self.y * self.y )
    end

    self.dot = function( v )
    	return self.x * v.x + self.y * v.y
    end

    self.det = function( v )
    	return self.x * v.y - self.y * v.x
    end

    self.rotate = function( r )
    	local x, y, c, s = self.x, self.y, cos( r ), sin( r )
    	self.x = x * c - y * s
    	self.y = x * s + y * c
    end

    self.angle = function()
    	return atan2( self.y,  self.x )
    end

    self.setLen = function( l )
    	local s = self.len()
    	if s > 0 then
    		s = l / s;
    		self.x = self.x * s
    		self.y = self.y * s
    	else
    		self.x = l
    		self.y = 0
    	end
    end

    self.normalize = function()
    	self.setLen( 1 )
    end

    -- Return the Vector2 object
	return self

end

return Vector2
