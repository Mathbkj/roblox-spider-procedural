--[[
	ProceduralSpider
	================
	A fully procedural, animation-free spider rig for Roblox.

	Nothing in this script relies on imported meshes or keyframed animations.
	The spider is assembled from primitive parts at runtime and every leg pose
	is solved each frame with FABRIK (Forward And Backward Reaching Inverse
	Kinematics). Feet are planted with raycasts, steps are planned with a
	distance threshold plus velocity prediction, legs move in an alternating
	diagonal gait, and the body tilts to match the plane defined by its feet.

	Pipeline per rendered frame:
		1. updateMovement - steer the body toward the player, fit it to the ground.
		2. updateLegs     - plan steps, animate swinging feet, solve IK chains.
		3. render         - write the solved positions into the visual parts.

	Place this LocalScript in StarterPlayer.StarterPlayerScripts.
]]

-- Tunable parameters. All distances are in studs, all times in seconds.
local CONFIG = { -- table that holds every tunable value of the spider
	LegPairs = 4, -- number of left/right leg pairs (4 pairs = 8 legs)
	Segments = { 2.6, 4, 4.6 }, -- length of each leg bone, from hip to foot
	BodyWidth = 3.2, -- size of the abdomen ellipsoid on the X axis
	BodyLength = 5.2, -- size of the abdomen ellipsoid on the Z axis
	BodyThickness = 2.2, -- size of the abdomen ellipsoid on the Y axis
	BodyHeight = 3.4, -- resting distance between the body and the ground plane
	LegSpread = 6.5, -- how far sideways the feet rest from the body center
	LegStretch = 1.35, -- multiplier that fans front/back feet further out than the hips
	Reach = 4, -- extra height above the body used as the raycast origin for feet

	Speed = 14, -- top walking speed
	TurnSpeed = 6, -- how quickly the heading turns toward the target
	FollowDistance = 12, -- the spider stops walking when the player is this close
	HeightSmoothing = 10, -- damping rate for the body height correction
	TiltSmoothing = 8, -- damping rate for the body tilt (surface normal)

	StepDistance = 2.6, -- a planted foot steps once it drifts this far from its ideal spot
	ForcedStepDistance = 6.5, -- beyond this distance a foot steps even if the gait says wait
	StepDuration = 0.16, -- time a single step takes from lift-off to touch-down
	StepHeight = 1.6, -- peak height of the foot arc during a step
	VelocityPrediction = 1.2, -- how much the foot target is pushed ahead along the velocity

	Iterations = 12, -- maximum FABRIK passes per leg per frame
	Tolerance = 0.02, -- FABRIK stops early when the foot is this close to the target
	KneeLift = 3, -- height of the pole point that keeps the knees bent upward
	PoleWeight = 0.35, -- how strongly inner joints are pulled toward the pole before solving

	BodyColor = Color3.fromRGB(30, 30, 38), -- color of the abdomen and head
	LegColor = Color3.fromRGB(45, 45, 55), -- color of the leg bones
	JointColor = Color3.fromRGB(255, 120, 40), -- glowing color of the knee and foot joints
	EyeColor = Color3.fromRGB(255, 60, 60), -- glowing color of the eyes
	LegThickness = 0.45, -- base thickness of the leg cylinders and joint spheres
	Debug = false, -- shows green markers at every foot's ideal target when true
} -- end of CONFIG

-- Services and the local player that the spider follows.
local Players = game:GetService("Players") -- service used to read the local player and other characters
local RunService = game:GetService("RunService") -- service that provides the per-frame RenderStepped event

local player = Players.LocalPlayer -- the player this client belongs to; the spider follows their character

-- Runtime state of a single leg.
type Leg = { -- type definition for the data stored per leg
	side: number, -- -1 for a left leg, 1 for a right leg
	group: number, -- gait group (0 or 1); legs of the same group step together
	hipLocal: Vector3, -- where the leg is attached, in body space
	restLocal: Vector3, -- where the foot prefers to stand, in body space
	joints: { Vector3 }, -- world positions of every joint, reused as the next IK starting guess
	planted: Vector3, -- where the foot is anchored on the ground
	foot: Vector3, -- where the foot is drawn this frame (differs from planted mid-step)
	stepping: boolean, -- true while the foot is swinging through the air
	stepFrom: Vector3, -- foot position when the current step started
	stepTo: Vector3, -- foot position the current step is heading to
	stepAlpha: number, -- progress of the current step from 0 to 1
	jointParts: { BasePart }, -- sphere parts drawn at each joint
	segmentParts: { BasePart }, -- cylinder parts drawn for each bone
	marker: BasePart?, -- optional debug sphere that shows the ideal foot target
} -- end of Leg

-- Runtime state of the whole creature.
type Spider = { -- type definition for the data stored for the spider as a whole
	model: Model, -- container model that holds every part
	body: BasePart, -- abdomen ellipsoid
	head: BasePart, -- head ellipsoid
	eyes: { BasePart }, -- the two eye spheres
	eyeOffsets: { CFrame }, -- cached offsets of each eye relative to the head
	stepCounts: { [number]: number }, -- how many legs of each gait group are currently in the air
	renderParts: { BasePart }, -- every moving part, in a fixed order, for BulkMoveTo
	renderFrames: { CFrame }, -- reusable array of target CFrames, same order as renderParts
	legs: { Leg }, -- all legs of the spider
	position: Vector3, -- world position of the body center
	up: Vector3, -- current up vector of the body (the ground normal, smoothed)
	heading: Vector3, -- direction the body is facing
	velocity: Vector3, -- current walking velocity
	frame: CFrame, -- body CFrame rebuilt from position, up and heading every frame
} -- end of Spider

-- Precomputed constants that would otherwise be rebuilt every frame.
local SEGMENT_ROTATION = CFrame.Angles(0, math.rad(90), 0) -- turns a lookAt frame (-Z forward) into a cylinder frame (X forward)
local HEAD_OFFSET = CFrame.new(0, 0.2, -CONFIG.BodyLength * 0.5 - CONFIG.BodyWidth * 0.25) -- head position relative to the body

-- Total leg length, cached once so the IK solver can detect unreachable targets cheaply.
local SEGMENT_TOTAL = 0 -- running sum of every bone length
for _, length in CONFIG.Segments do -- walk through each bone length
	SEGMENT_TOTAL += length -- add the bone to the total reach of a leg
end -- end of bone length loop

-- A single RaycastParams object is reused for every foot raycast.
local rayParams = RaycastParams.new() -- created once instead of once per leg per frame
rayParams.FilterType = Enum.RaycastFilterType.Exclude -- the filter list below lists things to ignore
rayParams.IgnoreWater = true -- feet stand on the terrain under water, not on the water surface

-- Rebuilds the raycast exclusion list so feet never stand on the spider itself or on players.
local function refreshFilter(spider: Spider) -- takes the spider whose parts must be ignored
	local ignore: { Instance } = { spider.model } -- always ignore the spider's own parts
	for _, other in Players:GetPlayers() do -- go through every player in the server
		if other.Character then -- skip players that have no character spawned
			table.insert(ignore, other.Character) -- ignore that character so feet never land on it
		end -- end of character check
	end -- end of player loop
	rayParams.FilterDescendantsInstances = ignore -- apply the new exclusion list to the shared params
end -- end of refreshFilter

-- Creates a purely visual part that never interferes with physics or raycasts.
local function makePart(name: string, shape: Enum.PartType, color: Color3, material: Enum.Material, parent: Instance): Part -- returns the new part
	local part = Instance.new("Part") -- create the part
	part.Name = name -- name it so it is easy to find in the Explorer
	part.Shape = shape -- block, ball or cylinder
	part.Color = color -- apply the requested color
	part.Material = material -- apply the requested material
	part.Anchored = true -- the script positions it directly, physics must not move it
	part.CanCollide = false -- nothing can bump into it
	part.CanQuery = false -- raycasts and spatial queries ignore it
	part.CanTouch = false -- it never fires Touched events
	part.TopSurface = Enum.SurfaceType.Smooth -- remove legacy studs on the top face
	part.BottomSurface = Enum.SurfaceType.Smooth -- remove legacy inlets on the bottom face
	part.Parent = parent -- place it inside the spider model
	return part -- hand the part back to the caller
end -- end of makePart

-- Creates an ellipsoid: a block whose sphere mesh stretches to match the part Size.
local function makeEllipsoid(name: string, size: Vector3, color: Color3, parent: Instance): Part -- returns the new ellipsoid
	local part = makePart(name, Enum.PartType.Block, color, Enum.Material.SmoothPlastic, parent) -- base block part
	part.Size = size -- the size defines the three radii of the ellipsoid
	local mesh = Instance.new("SpecialMesh") -- mesh that changes how the block is drawn
	mesh.MeshType = Enum.MeshType.Sphere -- draw the block as a sphere stretched to its size
	mesh.Parent = part -- attach the mesh to the part
	return part -- hand the ellipsoid back to the caller
end -- end of makeEllipsoid

-- Removes the component of a vector that points along a plane normal.
local function projectOnPlane(vector: Vector3, normal: Vector3): Vector3 -- returns the projected vector
	return vector - normal * vector:Dot(normal) -- subtract the part of the vector that is parallel to the normal
end -- end of projectOnPlane

-- Normalizes a vector, or returns a fallback when the vector is too short to normalize safely.
local function safeUnit(vector: Vector3, fallback: Vector3): Vector3 -- returns a unit vector
	return if vector.Magnitude > 1e-5 then vector.Unit else fallback -- avoid NaN from normalizing a zero vector
end -- end of safeUnit

-- Evaluates a 3-point Bezier curve, used for the arc of a stepping foot.
local function quadraticBezier(a: Vector3, control: Vector3, b: Vector3, t: number): Vector3 -- returns the point at time t
	local u = 1 - t -- complement of t, used by the Bernstein weights
	return a * (u * u) + control * (2 * u * t) + b * (t * t) -- weighted blend of start, control and end points
end -- end of quadraticBezier

-- Eases a 0..1 value in and out so steps accelerate and land softly.
local function smoothStep(t: number): number -- returns the eased value
	return t * t * (3 - 2 * t) -- classic smoothstep polynomial: zero slope at both ends
end -- end of smoothStep

-- Converts a rate into a lerp factor that behaves the same at any frame rate.
local function damp(rate: number, dt: number): number -- returns a blend factor between 0 and 1
	return 1 - math.exp(-rate * dt) -- exponential decay keeps 30 FPS and 240 FPS identical
end -- end of damp

-- FABRIK inverse kinematics solver. Moves the joints in place so the chain starts at
-- root, ends as close as possible to target, keeps every bone length and bends toward pole.
local function solveFabrik(joints: { Vector3 }, root: Vector3, target: Vector3, pole: Vector3, up: Vector3) -- solves one leg chain
	local count = #joints -- number of joints in the chain (bones + 1)
	local lengths = CONFIG.Segments -- bone lengths, indexed by the joint they start at
	local reach = target - root -- vector from the hip to the desired foot position
	if reach.Magnitude >= SEGMENT_TOTAL then -- the target is out of reach for the whole leg
		local direction = safeUnit(reach, -up) -- direction from the hip toward the target
		joints[1] = root -- the first joint always sits on the hip
		for index = 2, count do -- place every following joint
			joints[index] = joints[index - 1] + direction * lengths[index - 1] -- straight line, one bone after another
		end -- end of straight-line layout
		return -- nothing else to solve, the leg is fully stretched
	end -- end of out-of-reach case

	for index = 2, count - 1 do -- every inner joint (not the hip, not the foot)
		joints[index] = joints[index]:Lerp(pole, CONFIG.PoleWeight) -- nudge it toward the pole so knees bend up and out
	end -- end of pole bias

	for _ = 1, CONFIG.Iterations do -- refine the chain a limited number of times
		joints[count] = target -- backward pass: pin the foot onto the target
		for index = count - 1, 1, -1 do -- walk from the foot back toward the hip
			local direction = safeUnit(joints[index] - joints[index + 1], up) -- direction from the child joint to this joint
			joints[index] = joints[index + 1] + direction * lengths[index] -- restore the bone length along that direction
		end -- end of backward pass
		joints[1] = root -- forward pass: pin the hip back in place
		for index = 2, count do -- walk from the hip out toward the foot
			local direction = safeUnit(joints[index] - joints[index - 1], -up) -- direction from the parent joint to this joint
			joints[index] = joints[index - 1] + direction * lengths[index - 1] -- restore the bone length along that direction
		end -- end of forward pass
		if (joints[count] - target).Magnitude <= CONFIG.Tolerance then -- the foot is close enough to the target
			break -- stop iterating early to save time
		end -- end of convergence check
	end -- end of iteration loop
end -- end of solveFabrik

-- Casts a ray and returns the hit position, or nil when nothing was hit.
local function findGround(origin: Vector3, direction: Vector3): Vector3? -- returns the ground point, if any
	local result = workspace:Raycast(origin, direction, rayParams) -- cast using the shared exclusion list
	return if result then result.Position else nil -- only the hit position matters to the caller
end -- end of findGround

-- Computes where a foot would like to stand right now.
local function idealFoot(spider: Spider, leg: Leg): Vector3 -- returns the ideal foot position in world space
	local frame = spider.frame -- current body CFrame
	local up = spider.up -- current body up vector
	local rest = frame:PointToWorldSpace(leg.restLocal) -- preferred foot position converted to world space
	local prediction = projectOnPlane(spider.velocity, up) * CONFIG.VelocityPrediction * CONFIG.StepDuration -- push the target ahead of the motion
	local origin = rest + up * (CONFIG.BodyHeight + CONFIG.Reach) + prediction -- start the ray above the predicted spot
	local length = (CONFIG.BodyHeight + CONFIG.Reach) * 2.5 -- long enough to reach down slopes and steps
	return findGround(origin, -up * length) or (rest + prediction) -- ground point, or the unprojected spot if nothing was hit
end -- end of idealFoot

-- Creates one leg: hip and rest positions, gait group, visual parts and the planted foot.
local function buildLeg(spider: Spider, side: number, pairIndex: number): Leg -- returns the new leg
	local legPairs = CONFIG.LegPairs -- how many pairs share the body length
	local along = if legPairs > 1 then (pairIndex - 1) / (legPairs - 1) else 0.5 -- 0 for the front pair, 1 for the back pair
	local z = (along - 0.5) * CONFIG.BodyLength * 0.8 -- spread the hips along the body, centered on it
	local hipLocal = Vector3.new(side * CONFIG.BodyWidth * 0.45, 0, z) -- hip on the side of the abdomen
	local restLocal = Vector3.new(side * CONFIG.LegSpread, -CONFIG.BodyHeight, z * CONFIG.LegStretch) -- foot rests out to the side and on the ground

	local leg: Leg = { -- initial state of the leg
		side = side, -- remember which side the leg is on
		group = (pairIndex + (if side > 0 then 1 else 0)) % 2, -- alternate groups so diagonal legs step together
		hipLocal = hipLocal, -- hip in body space
		restLocal = restLocal, -- rest foot in body space
		joints = {}, -- filled below
		planted = Vector3.zero, -- set once the foot is placed on the ground
		foot = Vector3.zero, -- set once the foot is placed on the ground
		stepping = false, -- the leg starts on the ground
		stepFrom = Vector3.zero, -- unused until the first step
		stepTo = Vector3.zero, -- unused until the first step
		stepAlpha = 0, -- unused until the first step
		jointParts = {}, -- filled below
		segmentParts = {}, -- filled below
		marker = nil, -- only created in debug mode
	} -- end of leg table

	local hip = spider.frame:PointToWorldSpace(hipLocal) -- hip position in world space
	for index = 1, #CONFIG.Segments + 1 do -- one joint per bone plus the foot tip
		leg.joints[index] = hip -- all joints start at the hip; the first IK solve unfolds them
		local joint = makePart(`Joint{index}`, Enum.PartType.Ball, CONFIG.JointColor, Enum.Material.Neon, spider.model) -- glowing joint sphere
		joint.Size = Vector3.one * CONFIG.LegThickness * (if index == #CONFIG.Segments + 1 then 1 else 1.5) -- the foot tip is smaller than the knees
		table.insert(leg.jointParts, joint) -- keep the part so it can be moved every frame
	end -- end of joint creation
	for index = 1, #CONFIG.Segments do -- one cylinder per bone
		local segment = makePart(`Segment{index}`, Enum.PartType.Cylinder, CONFIG.LegColor, Enum.Material.Metal, spider.model) -- metallic bone cylinder
		segment.Size = Vector3.new(CONFIG.Segments[index], CONFIG.LegThickness * (1.25 - index * 0.15), CONFIG.LegThickness * (1.25 - index * 0.15)) -- bone length on X, thinner toward the foot
		table.insert(leg.segmentParts, segment) -- keep the part so it can be moved every frame
	end -- end of bone creation
	if CONFIG.Debug then -- debug markers are optional
		local marker = makePart("Target", Enum.PartType.Ball, Color3.fromRGB(80, 255, 120), Enum.Material.Neon, spider.model) -- green sphere
		marker.Size = Vector3.one * 0.5 -- small enough not to hide the foot
		marker.Transparency = 0.3 -- slightly see-through
		leg.marker = marker -- updateLegs moves it onto the ideal target every frame
	end -- end of debug marker

	local start = idealFoot(spider, leg) -- find the ground under the rest position
	leg.planted = start -- anchor the foot there
	leg.foot = start -- and draw it there
	return leg -- hand the finished leg back to buildSpider
end -- end of buildLeg

-- Assembles the whole spider and snaps it to the ground below the requested origin.
local function buildSpider(origin: CFrame): Spider -- returns the new spider
	local model = Instance.new("Model") -- container for every part of the spider
	model.Name = "ProceduralSpider" -- name shown in the Explorer

	local body = makeEllipsoid("Body", Vector3.new(CONFIG.BodyWidth, CONFIG.BodyThickness, CONFIG.BodyLength), CONFIG.BodyColor, model) -- abdomen
	local head = makeEllipsoid("Head", Vector3.new(CONFIG.BodyWidth * 0.7, CONFIG.BodyThickness * 0.75, CONFIG.BodyWidth * 0.7), CONFIG.BodyColor, model) -- head
	local eyes: { BasePart } = {} -- eye parts
	local eyeOffsets: { CFrame } = {} -- eye offsets relative to the head
	for _, side in { -1, 1 } do -- left eye, then right eye
		local eye = makePart("Eye", Enum.PartType.Ball, CONFIG.EyeColor, Enum.Material.Neon, model) -- glowing eye sphere
		eye.Size = Vector3.one * 0.45 -- small sphere
		table.insert(eyes, eye) -- keep the part so it can be moved every frame
		table.insert(eyeOffsets, CFrame.new(side * CONFIG.BodyWidth * 0.17, 0.25, -CONFIG.BodyWidth * 0.3)) -- cached so it is not rebuilt every frame
	end -- end of eye creation
	model.PrimaryPart = body -- the body is the reference part of the model
	model.Parent = workspace -- make the spider visible

	local spider: Spider = { -- initial state of the spider
		model = model, -- container model
		body = body, -- abdomen part
		head = head, -- head part
		eyes = eyes, -- eye parts
		eyeOffsets = eyeOffsets, -- eye offsets
		stepCounts = { [0] = 0, [1] = 0 }, -- no leg is in the air yet
		renderParts = {}, -- filled after the legs exist
		renderFrames = {}, -- filled after the legs exist
		legs = {}, -- filled below
		position = origin.Position, -- start at the requested origin
		up = Vector3.yAxis, -- assume flat ground until the feet say otherwise
		heading = safeUnit(projectOnPlane(origin.LookVector, Vector3.yAxis), -Vector3.zAxis), -- face the origin's horizontal direction
		velocity = Vector3.zero, -- start standing still
		frame = origin, -- temporary frame until it is rebuilt below
	} -- end of spider table
	refreshFilter(spider) -- make sure the first raycasts ignore the spider and the players

	local ground = findGround(origin.Position + Vector3.yAxis * 10, -Vector3.yAxis * 200) -- look for ground below the spawn point
	if ground then -- ground was found
		spider.position = ground + Vector3.yAxis * CONFIG.BodyHeight -- start at the correct height above it
	end -- end of ground check
	spider.frame = CFrame.lookAt(spider.position, spider.position + spider.heading) -- body frame used to place the legs

	for pairIndex = 1, CONFIG.LegPairs do -- every pair from front to back
		for _, side in { -1, 1 } do -- left leg, then right leg
			table.insert(spider.legs, buildLeg(spider, side, pairIndex)) -- build the leg and store it
		end -- end of side loop
	end -- end of pair loop

	local renderParts = spider.renderParts -- flat, ordered list consumed by BulkMoveTo
	table.insert(renderParts, body) -- slot 1: body
	table.insert(renderParts, head) -- slot 2: head
	for _, eye in eyes do -- next slots: eyes
		table.insert(renderParts, eye) -- add the eye
	end -- end of eye slots
	for _, leg in spider.legs do -- remaining slots: every leg in order
		for _, part in leg.jointParts do -- the leg's joint spheres
			table.insert(renderParts, part) -- add the joint
		end -- end of joint slots
		for _, part in leg.segmentParts do -- the leg's bone cylinders
			table.insert(renderParts, part) -- add the bone
		end -- end of bone slots
	end -- end of leg slots
	for index, part in renderParts do -- pre-size the CFrame array
		spider.renderFrames[index] = part.CFrame -- start with the current CFrame so the array is never sparse
	end -- end of pre-size loop
	return spider -- hand the finished spider back
end -- end of buildSpider

-- Estimates the ground plane under the spider from its planted feet.
local function surfaceNormal(spider: Spider): Vector3 -- returns the estimated ground normal
	local left, right, front, back = Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero -- position sums per foot group
	local leftCount, rightCount, frontCount, backCount = 0, 0, 0, 0 -- how many feet each sum contains
	for _, leg in spider.legs do -- classify every planted foot
		if leg.side < 0 then -- left leg
			left += leg.planted -- add to the left sum
			leftCount += 1 -- count it
		else -- right leg
			right += leg.planted -- add to the right sum
			rightCount += 1 -- count it
		end -- end of side classification
		if leg.restLocal.Z < 0 then -- leg in the front half of the body
			front += leg.planted -- add to the front sum
			frontCount += 1 -- count it
		elseif leg.restLocal.Z > 0 then -- leg in the back half of the body
			back += leg.planted -- add to the back sum
			backCount += 1 -- count it
		end -- end of front/back classification
	end -- end of foot loop
	if leftCount == 0 or rightCount == 0 or frontCount == 0 or backCount == 0 then -- not enough feet to define a plane
		return spider.up -- keep the current up vector
	end -- end of degenerate case
	local across = right / rightCount - left / leftCount -- vector across the body, lying on the ground plane
	local forward = front / frontCount - back / backCount -- vector along the body, lying on the ground plane
	local normal = across:Cross(forward) -- perpendicular to both, so perpendicular to the ground
	if normal:Dot(spider.up) < 0 then -- the cross product points into the ground
		normal = -normal -- flip it so it points away from the ground
	end -- end of orientation fix
	return safeUnit(normal, spider.up) -- normalized ground normal
end -- end of surfaceNormal

-- Average position of all planted feet, used to keep the body at a constant height.
local function feetCenter(spider: Spider): Vector3 -- returns the centroid of the feet
	local sum = Vector3.zero -- running sum of foot positions
	for _, leg in spider.legs do -- every leg
		sum += leg.planted -- add its anchored foot
	end -- end of foot loop
	return sum / #spider.legs -- divide by the number of feet to get the average
end -- end of feetCenter

-- True when any leg of the given gait group is currently in the air.
local function groupStepping(spider: Spider, group: number): boolean -- returns whether that group is busy
	return spider.stepCounts[group] > 0 -- O(1): counters are updated when steps start and end
end -- end of groupStepping

-- Moves and orients the body: steer, tilt to the ground, keep height, rebuild the frame.
local function updateMovement(spider: Spider, dt: number) -- dt is the clamped frame time
	local character = player.Character -- the character to follow
	local root = character and character:FindFirstChild("HumanoidRootPart") -- its root part, if it exists
	local desired = Vector3.zero -- velocity the spider wants; zero means stand still
	if root and root:IsA("BasePart") then -- only steer when there is a valid target
		local offset = projectOnPlane(root.Position - spider.position, spider.up) -- direction to the player along the ground
		if offset.Magnitude > CONFIG.FollowDistance then -- the player is too far away
			desired = offset.Unit * CONFIG.Speed -- walk toward them at full speed
		end -- end of follow check
		if offset.Magnitude > 1 then -- avoid turning when standing on top of the player
			local look = offset.Unit -- direction the spider should face
			spider.heading = safeUnit(spider.heading:Lerp(look, damp(CONFIG.TurnSpeed, dt)), look) -- turn smoothly toward it
		end -- end of turn check
	end -- end of target check
	spider.velocity = spider.velocity:Lerp(desired, damp(5, dt)) -- accelerate and brake smoothly
	spider.position += spider.velocity * dt -- integrate the position

	local normal = surfaceNormal(spider) -- ground normal under the feet
	spider.up = safeUnit(spider.up:Lerp(normal, damp(CONFIG.TiltSmoothing, dt)), Vector3.yAxis) -- tilt smoothly toward it

	local center = feetCenter(spider) -- average foot position
	local wanted = center + spider.up * CONFIG.BodyHeight -- where the body should hover
	local correction = (wanted - spider.position):Dot(spider.up) -- height error measured only along the up axis
	spider.position += spider.up * correction * damp(CONFIG.HeightSmoothing, dt) -- correct the height without fighting the walk

	local forward = safeUnit(projectOnPlane(spider.heading, spider.up), spider.frame.LookVector) -- facing direction flattened onto the ground plane
	local rightVector = forward:Cross(spider.up) -- right vector perpendicular to forward and up
	spider.frame = CFrame.fromMatrix(spider.position, rightVector, spider.up) -- final orthonormal body frame
end -- end of updateMovement

-- Plans steps, animates feet that are in the air and solves the IK of every leg.
local function updateLegs(spider: Spider, dt: number) -- dt is the clamped frame time
	for _, leg in spider.legs do -- process each leg independently
		local ideal = idealFoot(spider, leg) -- where this foot would like to stand right now
		local marker = leg.marker -- debug marker, if enabled
		if marker then -- debug mode is on
			marker.CFrame = CFrame.new(ideal) -- show the ideal target
		end -- end of debug marker update

		if leg.stepping then -- the foot is in the air
			leg.stepAlpha = math.min(leg.stepAlpha + dt / CONFIG.StepDuration, 1) -- advance the step progress
			leg.stepTo = leg.stepTo:Lerp(ideal, damp(12, dt)) -- keep tracking the ideal spot while the body moves
			local alpha = smoothStep(leg.stepAlpha) -- eased progress for a soft lift and landing
			local middle = (leg.stepFrom + leg.stepTo) / 2 + spider.up * CONFIG.StepHeight * 2 -- control point above the midpoint
			leg.foot = quadraticBezier(leg.stepFrom, middle, leg.stepTo, alpha) -- position on the arc
			if leg.stepAlpha >= 1 then -- the step has finished
				leg.stepping = false -- the foot is back on the ground
				spider.stepCounts[leg.group] -= 1 -- one less leg of this group in the air
				leg.planted = leg.stepTo -- anchor the foot where it landed
				leg.foot = leg.stepTo -- draw it there
			end -- end of touch-down
		else -- the foot is on the ground
			local distance = (leg.planted - ideal).Magnitude -- how far the anchor drifted from the ideal spot
			local otherGroupBusy = groupStepping(spider, 1 - leg.group) -- is the opposite group in the air?
			if distance > CONFIG.ForcedStepDistance or (distance > CONFIG.StepDistance and not otherGroupBusy) then -- time to step
				leg.stepping = true -- lift the foot
				spider.stepCounts[leg.group] += 1 -- one more leg of this group in the air
				leg.stepAlpha = 0 -- restart the step progress
				leg.stepFrom = leg.planted -- the arc starts at the current anchor
				leg.stepTo = ideal -- and heads to the ideal spot
			end -- end of step decision
			leg.foot = leg.planted -- a grounded foot stays on its anchor
		end -- end of stepping branch

		local hip = spider.frame:PointToWorldSpace(leg.hipLocal) -- hip in world space for this frame
		local outward = safeUnit(projectOnPlane(leg.foot - hip, spider.up), spider.frame.RightVector * leg.side) -- horizontal direction from hip to foot
		local pole = hip + outward * CONFIG.Segments[1] + spider.up * CONFIG.KneeLift -- point above and outside the hip that the knees bend toward
		solveFabrik(leg.joints, hip, leg.foot, pole, spider.up) -- solve the chain from the hip to the foot
	end -- end of leg loop
end -- end of updateLegs

-- Copies the simulated state onto the visual parts with a single BulkMoveTo call.
local function render(spider: Spider) -- draws the current state
	local frames = spider.renderFrames -- reusable CFrame array, same order as renderParts
	local cursor = 1 -- next slot to write
	local frame = spider.frame -- body CFrame
	frames[cursor] = frame -- slot 1: body
	cursor += 1 -- move to the next slot
	local headFrame = frame * HEAD_OFFSET -- head follows the body with a fixed offset
	frames[cursor] = headFrame -- slot 2: head
	cursor += 1 -- move to the next slot
	for _, offset in spider.eyeOffsets do -- every eye
		frames[cursor] = headFrame * offset -- eye follows the head with its cached offset
		cursor += 1 -- move to the next slot
	end -- end of eye slots
	for _, leg in spider.legs do -- every leg, in the same order used when building renderParts
		local joints = leg.joints -- solved joint positions of this leg
		for index = 1, #leg.jointParts do -- every joint sphere
			frames[cursor] = CFrame.new(joints[index]) -- sphere sits on its joint
			cursor += 1 -- move to the next slot
		end -- end of joint slots
		for index = 1, #leg.segmentParts do -- every bone cylinder
			local a = joints[index] -- joint where the bone starts
			local b = joints[index + 1] -- joint where the bone ends
			if (b - a).Magnitude > 1e-4 then -- skip degenerate bones whose ends overlap
				frames[cursor] = CFrame.lookAt((a + b) * 0.5, b) * SEGMENT_ROTATION -- centered between the joints, X axis along the bone
			end -- end of degenerate check
			cursor += 1 -- move to the next slot even when the bone was skipped
		end -- end of bone slots
	end -- end of leg slots
	workspace:BulkMoveTo(spider.renderParts, frames, Enum.BulkMoveMode.FireCFrameChanged) -- move every part in one engine call
end -- end of render

-- Yields until the local character exists and returns its HumanoidRootPart.
local function waitForRoot(): BasePart -- returns the character's root part
	local character = player.Character or player.CharacterAdded:Wait() -- use the current character or wait for one
	return character:WaitForChild("HumanoidRootPart") :: BasePart -- wait until the root part has replicated
end -- end of waitForRoot

local root = waitForRoot() -- the player's root part at startup
local spider = buildSpider(root.CFrame * CFrame.new(0, 0, -CONFIG.FollowDistance - 6)) -- spawn the spider in front of the player

local filterClock = 0 -- time since the raycast filter was last refreshed
RunService.RenderStepped:Connect(function(dt) -- runs once per rendered frame, before the frame is drawn
	local step = math.min(dt, 1 / 20) -- clamp the frame time so a hitch never makes the simulation jump
	filterClock += dt -- accumulate real time for the filter refresh
	if filterClock > 1 then -- once per second
		filterClock = 0 -- reset the timer
		refreshFilter(spider) -- pick up characters that spawned or respawned
	end -- end of filter refresh
	updateMovement(spider, step) -- move and orient the body
	updateLegs(spider, step) -- plan steps and solve every leg
	render(spider) -- draw the result
end) -- end of the main loop