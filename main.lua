


-->==========[ CONFIG ]==========<--
-- Properties
local RADIUS = 1

-- Camera
local CAM_MAX_DIP = 0.75 -- maximum camera pitch in percentage
local CAM_TOLERANCE = 3 * RADIUS
local CAM_DISTANCE = 5 * RADIUS

-- Physics
local GRAVITY = vec(0,-0.03,0)
local FRICTION = 0.2
local COLLISION_MARGIN = 0.0001
local CONTROL_SPEED = 1
local MAX_COLLISIONS = 100

-- Model
local MODEL_BASE = models.ball
local MODEL_PLAYER = models.ball.plaer
local MODEL_BALL = models.ball.ball

-- Advanced (best left untouched)
local CHECK_MARGIN = 0.5
local LOW_DETAIL_MODE = false
local MOVE_PLAYER = false -- Requires host:setPos()
-->==============================<--

local lpos = vec(0,0,0)
local pos = vec(0,0,0)
local vel = vec(0,0,0)
local rot = matrices.mat3()
local rvel = vec(0,0,0)


---projects dir into the normal
---@param vector Vector3
---@param normal Vector3
local function flatten(vector, normal)
	normal = normal:normalized()
	return vector - vector:dot( normal) * normal
end



events.ENTITY_INIT:register(function ()
   pos = player:getPos():add(0,RADIUS)
   
   MODEL_BASE:setParentType("WORLD")
   vanilla_model.ALL:setVisible(false)
   local pscale = 0.9 * RADIUS
   MODEL_PLAYER
   :setPrimaryTexture("SKIN")
   :setScale(pscale,pscale,pscale)
   if player:getModelType() == "SLIM" then
      MODEL_PLAYER.slim:setVisible(true)
      MODEL_PLAYER.base:setVisible(false)
   end
   MODEL_BALL
   :setPrimaryTexture("RESOURCE","minecraft:textures/block/glow_lichen.png")
end)

local isHost = host:isHost()
local control = vec(0,0,0,0)

local lcamDir = vec(0,0,0)
local camDir = vec(0,0,-0.01)

local blockChecks = {}
local collisionCache = {}
local function setRadius(radius)
   local lastRadius = RADIUS
   RADIUS = radius
   LOW_DETAIL_MODE = RADIUS > 5
   blockChecks = {}
   if LOW_DETAIL_MODE then
      local ray_count = 64
      for i = 1, ray_count, 1 do
         local t = i / ray_count
         local inclination = math.acos(1 - 2 * t)
         local azimuth = 2 * math.pi * 0.618033 * i
         local dir = vectors.vec3(
            math.sin(inclination) * math.sin(azimuth),
            math.cos(inclination),
            math.sin(inclination) * math.cos(azimuth)
         )
         blockChecks[i] = dir * RADIUS
      end
   else
      local r = math.max(RADIUS+CHECK_MARGIN,2)
      local ra = r^2
      for z = -r, r, 1 do
         for y = -r, r, 1 do
            for x = -r, r, 1 do
               local fpos = vec(x+0.5,y+0.5,z+0.5)
               local len = fpos:lengthSquared()
               if len < ra then
                  blockChecks[#blockChecks+1] = fpos
               end
            end
         end
      end
   end
   
   MODEL_BALL
   :setScale(RADIUS,RADIUS,RADIUS)
   local pscale = 0.9 * RADIUS
   MODEL_PLAYER
   :setPrimaryTexture("SKIN")
   :setScale(pscale,pscale,pscale)
   CAM_DISTANCE = (CAM_DISTANCE / lastRadius) * radius
   CAM_TOLERANCE = (CAM_TOLERANCE / lastRadius) * radius
   GRAVITY = (GRAVITY / math.min(lastRadius,1)) * math.min(radius,1)
   CONTROL_SPEED = (CONTROL_SPEED / lastRadius) * radius
end

setRadius(RADIUS)




local function sync()
   pings.control(control.x,control.y,control.z,camDir.x,camDir.y,camDir.z)
end

function pings.control(x,y,z,dx,dy,dz)
   if not isHost then
      control = vec(x,y,z)
      camDir = vec(dx,dy,dz)
   end
end

local function snap(a,step)
   return math.floor(a * step + 0.5) / step
end

function pings.state(px,py,pz, r1x,r1y,r1z,r2x,r2y,r2z, vx,vy,vz, rx,ry,rz, radius)
   if not isHost then
      pos = vec(px,py,pz)
      vel = vec(vx,vy,vz)
      local r1 = vec(r1x,r1y,r1z):normalize()
      local r2 = vec(r2x,r2y,r2z):normalize()
      rot = matrices.mat3(r1,r2,r1:copy():cross(r2))
      rvel = vec(rx,ry,rz)
      if radius ~= RADIUS then
         setRadius(radius)
      end
   end
end


if isHost then
   local f3 = keybinds:newKeybind("F3","key.keyboard.f3")
   local keys = {
      forward = keybinds:fromVanilla("key.forward"),
      backward = keybinds:fromVanilla("key.back"),
      left = keybinds:fromVanilla("key.left"),
      right = keybinds:fromVanilla("key.right"),
      jump = keybinds:fromVanilla("key.jump"),
   }
   keys.forward.press = function () control.z = control.z + 1 sync() return true end
   keys.forward.release = function () control.z = control.z - 1 sync() end
   
   keys.backward.press = function () control.z = control.z - 1 sync() return true end
   keys.backward.release = function () control.z = control.z + 1 sync() end
   
   keys.left.press = function () control.x = control.x - 1 sync() return true end
   keys.left.release = function () control.x = control.x + 1 sync() end
   
   keys.right.press = function () control.x = control.x + 1 sync() return not f3:isPressed() end -- lmao
   keys.right.release = function () control.x = control.x - 1 sync() end
   
   keys.jump.press = function () control.y = control.y + 1 sync() return true end
   keys.jump.release = function () control.y = control.y - 1 sync() end
end

local syncTimer = 0
events.WORLD_TICK:register(function()
   if isHost then
      syncTimer = syncTimer + 1
      if syncTimer > 20 then
         syncTimer = 0
         pings.state(
            snap(pos.x,100),
            snap(pos.y,100),
            snap(pos.z,100),
            snap(rot[1].x,10000),
            snap(rot[1].y,10000),
            snap(rot[1].z,10000),
            snap(rot[2].x,10000),
            snap(rot[2].y,10000),
            snap(rot[2].z,10000),
            snap(vel.x,100),
            snap(vel.y,100),
            snap(vel.z,100),
            snap(rvel.x,100),
            snap(rvel.y,100),
            snap(rvel.z,100),
            RADIUS
         )
      end
   end
   lpos = pos
   
   -- Camera
   if isHost then
      lcamDir = camDir
   end
   camDir = (((camDir + vec(0,0.1,0))* CAM_TOLERANCE) - vel)
   camDir.y = math.clamp(camDir.y,-CAM_MAX_DIP,CAM_MAX_DIP)

   camDir:normalize()
   
   local intersectionPoints = {}
   local samples = 0
   for i = 1, #blockChecks, 1 do
      local fpos = blockChecks[i]
      local block = world.getBlockState(fpos+pos)
      if block:hasCollision() then
         samples = samples + 1
         local AABB
         if LOW_DETAIL_MODE then
            AABB = {{vec(0,0,0),vec(1,1,1)}}
         else
            AABB = block:getCollisionShape()
         end
         collisionCache[fpos] = block
         local offset = block:getPos()
         for key, box in pairs(AABB) do
            local a = box[1] + offset
            local b = box[2] + offset
            local clampedPos = vec(
               math.clamp(pos.x, a.x,b.x),
               math.clamp(pos.y, a.y,b.y),
               math.clamp(pos.z, a.z,b.z)
            )
            local dist = (clampedPos-pos):length()
            if RADIUS >= dist then
               intersectionPoints[#intersectionPoints+1] = clampedPos
            end
         end
      end
   end
   host:setActionbar("samples: "..samples)
   table.sort(intersectionPoints,function (a, b) return (a-pos):length() > (b-pos):length()end)
   local factor = 1/#intersectionPoints
   for i = 1, math.min(#intersectionPoints,MAX_COLLISIONS), 1 do
      local ipos = intersectionPoints[i]
      local dir = (pos-ipos):normalize()
      
      -- rotation
      local pointVel = rvel:copy():cross(dir)*0.05
      -- absorb rotation
      local torque = (dir*10):cross(pointVel+vel)
      rvel = rvel + (torque-rvel) * FRICTION * factor
      -- apply rotation
      vel = vel + (pointVel-vel) * FRICTION * factor
      
      -- postion
      local flatVel = flatten(vel, dir)
      local absorbed = (flatVel-vel)
      if absorbed:length() > 0.2 then
         sounds.impact:pos(pos):play()
      end
      vel = flatVel
      pos = ipos + dir * (RADIUS + COLLISION_MARGIN)
   end
   
   local omega = (rot * rot:transposed()) * (rvel:length() ~= 0 and rvel or vec(0,0.000001,0))
   local speed = math.deg(omega:length()) * 0.05 / RADIUS
   rot.c1 = vectors.rotateAroundAxis(speed,rot.c1,omega)
	rot.c2 = vectors.rotateAroundAxis(speed,rot.c2,omega)
	rot.c3 = vectors.rotateAroundAxis(speed,rot.c3,omega)
   
   -- Controls
   
---@diagnostic disable-next-line: deprecated
   local forward = camDir.x_z:normalize():cross(vec(0,1,0))
   local right = forward:copy():cross(vec(0,1,0))
   rvel = rvel
   + forward * control.z * CONTROL_SPEED
   + right * control.x * CONTROL_SPEED
   vel = vel + GRAVITY
   pos = pos + vel
   
   if isHost then
      if MOVE_PLAYER then
         local fpos = pos - vec(0,player:getBoundingBox().y*0.5,0)
---@diagnostic disable-next-line: undefined-field
         host:setPos(fpos)
         --host:sendChatCommand("/tp " .. snap(fpos.x,1000) .. " " .. snap(fpos.y,1000) .. " " .. snap(fpos.z,1000))
      end
   end
end)

renderer:setRenderCrosshair(false)

events.POST_WORLD_RENDER:register(function (deltaTick)
   local trot = rot:copy()
   local omega = (rot * rot:transposed()) * (rvel:length() ~= 0 and rvel or vec(0,0.000001,0))
   local speed = math.deg(omega:length()) * 0.05 * deltaTick / RADIUS
   trot.c1 = vectors.rotateAroundAxis(speed,trot.c1,omega)
	trot.c2 = vectors.rotateAroundAxis(speed,trot.c2,omega)
	trot.c3 = vectors.rotateAroundAxis(speed,trot.c3,omega)
   
   local transformation = trot:augmented():translate(math.lerp(lpos, pos, deltaTick) * 16)
   
   local tcamDir = math.lerp(lcamDir,camDir,deltaTick)
   renderer:setCameraPivot(transformation:apply() / 16 + tcamDir * CAM_DISTANCE)
   :setCameraRot(math.deg(math.asin(tcamDir.y)),math.deg(math.atan2(tcamDir.x,-tcamDir.z)))
   MODEL_BASE:setMatrix(transformation)
end)

if isHost then
   local function warn(msg)
      printJson('{"text":"'..msg..'","color":"yellow"}')
   end
   events.WORLD_RENDER:register(function (delta)
      if host:isChatOpen() and host:getChatText():find("^$resize") then
         host:setChatColor(1,1,0)
      else host:setChatColor(1,1,1)
      end
   end)
   
   events.CHAT_SEND_MESSAGE:register(function (message)
      if message:find("^$resize") then
         local num = tonumber(message:match("^$resize ([0-9.-]+)$"))
         if num then setRadius(num)
         else print("Invalid number given")
         end host:appendChatHistory(message) return ""
      end
      return message
   end)
end