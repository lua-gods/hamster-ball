--[[______   __                _                 __
  / ____/ | / /___ _____ ___  (_)___ ___  ____ _/ /____  _____
 / / __/  |/ / __ `/ __ `__ \/ / __ `__ \/ __ `/ __/ _ \/ ___/
/ /_/ / /|  / /_/ / / / / / / / / / / / / /_/ / /_/  __(__  )
\____/_/ |_/\__,_/_/ /_/ /_/_/_/ /_/ /_/\__,_/\__/\___/____]]
local smears = {}

local config = {
	world = models:newPart("worldTrail","WORLD"):setScale(-16,-16,16)
}

local white = textures["1x1white"] or textures:newTexture("1x1white",1,1):setPixel(0,0,vec(1,1,1))

-->====================[ API ]====================<--

---A trail type that is controlled by two world positions
---@class Trail
---@field ID integer
---@field leadA Vector3
---@field leadB Vector3
---@field lead_width number
---@field duration integer
---@field texture Texture
---@field points table
---@field sprites table
---@field render_type ModelPart.renderType
---@field sprites_flipped table
---@field diverge number
local Trail = {}
Trail.__index = Trail

local smearID = 0
---Creates a new Trail
---@param texture Texture?
---@return Trail
function Trail.new(texture)
	---@type Trail
	local self = {
		ID = smearID,
		leadA = nil,
		leadB = nil,
		lead_width = 1,
		texture=texture or white,
		duration = 20,
		points = {},
		render_type = "EMISSIVE",
		sprites = {},
		sprites_flipped = {},
		diverge = 1,
	}
	smearID = smearID + 1
	setmetatable(self,Trail)
	self:rebuildSpriteTasks()
	table.insert(smears,self)
	return self
end

---Sets the two points which the trail will follow  
---3rd agument defaults to 1 if none given
---@param A Vector3
---@param B Vector3
---@param scale number|nil
---@return Trail
function Trail:setLeads(A,B,scale)
	if not scale then scale = 1 end
	self.leadA = A:copy()
	self.leadB = B:copy()
	self.lead_width = scale
	self:update()
	return self
end

function Trail:clear()
	self.leadA = nil
	self.leadB = nil
	self.points = {}
	self:rebuildSpriteTasks()
	return self
end

---sets the divergeness index.  
---the index can be a decimal, for control over how much the effect applies.
---***
--- 0 : shrink  
--- 0.5 : shrink halfway  
--- 1 : none  
--- 1.5 : grow halfway  
--- 2 : grow  
---@param index number
---@return Trail
function Trail:setDivergeness(index)
	self.diverge = index
	return self
end

---Sets the duration of the trail, the duration is based on update ticks(not minecraft ticks).
---@param ticks integer
---@return Trail
function Trail:setDuration(ticks)
	self.duration = ticks
	self:rebuildSpriteTasks()
	return self
end

---Sets the render type of the smear.
---@param render_type ModelPart.renderType
---@return Trail
function Trail:setRenderType(render_type)
	self.render_type = render_type
	self:rebuildSpriteTasks()
	return self
end

---Deletes all the sprite tasks, must be called when discarding the object.
function Trail:free()
	for _, t in pairs(self.sprites) do config.world:removeTask(t:getName()) end
	for _, t in pairs(self.sprites_flipped) do config.world:removeTask(t:getName()) end
end

---Rebuilds the sprite tasks.
---@return Trail
function Trail:rebuildSpriteTasks()
	for _, t in pairs(self.sprites) do config.world:removeTask(t:getName()) end
	for _, t in pairs(self.sprites_flipped) do config.world:removeTask(t:getName()) end
	self.sprites = {}
	self.sprites_flipped = {}
	for i = 1, self.duration-1, 1 do
		local j,l = i/self.duration,(i+1)/self.duration
		local new = config.world:newSprite(self.ID.."GNSMEAR"..i):setTexture(self.texture):setRenderType(self.render_type)
		local v = new:getVertices()
		v[1]:uv(0,j) v[2]:uv(1,j) v[3]:uv(1,l)v[4]:uv(0,l)
		table.insert(self.sprites,new)
	end
	for i = 1, self.duration-1, 1 do
		local j,l = i/self.duration,(i+1)/self.duration
		local new = config.world:newSprite(self.ID.."GNSMEAR"..i.."FLIP"):setTexture(self.texture):setRenderType(self.render_type)
		local v = new:getVertices()
		v[2]:uv(0,j) v[1]:uv(1,j) v[4]:uv(1,l)v[3]:uv(0,l)
		table.insert(self.sprites_flipped,new)
	end
	return self
end

---Updates the Trail Rendering
---@return Trail
function Trail:update()
	if self.leadA and self.leadB then
		table.insert(self.points,1,{self.leadA,self.leadB,self.lead_width})
		while #self.points > self.duration do
			table.remove(self.points,self.duration+1)
		end
		
		for id = 1, #self.points-1, 1 do
			local invisible = ((self.points[id][3] + self.points[id+1][3]) == 0)
				self.sprites[id]:setVisible(not invisible)
				self.sprites_flipped[id]:setVisible(not invisible)
				if not invisible then
					local v = self.sprites[id]:getVertices()
					local v2 = self.sprites_flipped[id]:getVertices()
					local width, width_next
					width = 1-(math.map((id / self.duration),0,1,1,self.diverge) * self.points[id][3])
					width_next = 1-(math.map(((id + 1) / self.duration),0,1,1,self.diverge) * self.points[id + 1][3])
					local a, b, c, d = (self.points[id][1]), (self.points[id][2]), (self.points[id+1][1]), (self.points[id+1][2])
					local a2, b2, c2, d2 = 
					math.lerp(a,b,width*0.5), math.lerp(b,a,width*0.5), 
					math.lerp(d,c,width_next*0.5), math.lerp(c,d,width_next*0.5)
					local offset = (a2 + b2 + c2 + d2) * 0.25
					self.sprites[id]:setPos(offset * vec(-1, -1, 1))
					self.sprites_flipped[id]:setPos(offset * vec(-1, -1, 1))
					a2, b2, c2, d2 = a2 - offset, b2 - offset, c2 - offset, d2 - offset
					v[1]:setPos(a2) v[2]:setPos(b2)
					v[3]:setPos(c2) v[4]:setPos(d2)
					v2[2]:setPos(a2) v2[1]:setPos(b2)
					v2[3]:setPos(d2) v2[4]:setPos(c2)
				end
		end
	end
	return self
end

return Trail