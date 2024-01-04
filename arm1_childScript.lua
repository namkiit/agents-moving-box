-- Global variable for duration
duration = 2

function sysCall_init()
    -- Retrieve handles for various objects in the simulation environment
    RefPosition = sim.getObjectHandle("Reference")
    EndEffector = sim.getObjectHandle("EndEffector")
    Connector = sim.getObjectHandle("Connector")
    Proximity = sim.getObjectHandle("Proximity")
    Goal = sim.getObjectHandle("Goal")
    Box = sim.getObjectHandle("Box")
    via = sim.getObjectHandle("via")

    -- Get initial positions of objects
    position_pick = sim.getObjectPosition(Box, -1)
    position_goal = sim.getObjectPosition(Goal, -1)
    position_via = sim.getObjectPosition(via, -1)

    -- Adjust the pick position based on the box's dimensions
    position_pick[2] = position_pick[2] - 0.1

    -- Set the reference position to the 'via' point
    sim.setObjectPosition(RefPosition, -1, position_via)

    -- Define states for the finite state machine (FSM)
    state_start, state_via2pick, state_pick, state_pick2via = 0, 1, 2, 3
    state_via2goal, state_goal, state_goal2via, state_release = 4, 5, 6, 7
    state_stop, state_rotate = 8, 9

    -- Initialize FSM and set init time
    state = state_start
    t_f = 1
end

function sysCall_actuation()
    -- Get current simulation time
    t = sim.getSimulationTime()

    -- Update state based on the pick-and-place process
    state = singlePickandPlace()
end

function sysCall_sensing()
    -- Insert sensing-related code here
end

function sysCall_cleanup()
    -- Insert cleanup code here
end

function singlePickandPlace()
    -- State transitions based on timing
    if state == state_start and t >= t_f then
        transitionToState(state_via2pick, position_via, position_pick)
    elseif state == state_via2pick and t >= t_f then
        state = state_pick
    elseif state == state_pick and t >= t_f then
        transitionToState(state_pick2via, position_pick, position_via)
    elseif state == state_pick2via and t >= t_f then
        transitionToState(state_via2goal, position_via, position_goal)
    elseif state == state_via2goal and t >= t_f then
        state = state_release
    elseif state == state_release and t >= t_f then
        transitionToState(state_goal2via, position_goal, position_via)
    elseif state == state_goal2via and t >= t_f then
        state = state_stop
    elseif state == state_stop then
        state = state_rotate
    end

    -- Actions based on current state
    handleStateActions()

    return state
end

-- Helper function for state transitions
function transitionToState(newState, startPosition, endPosition)
    state = newState
    position_i, position_f = startPosition, endPosition
    t_i = t_f
    t_f = t_i + duration
end

-- Function to handle actions based on current state
function handleStateActions()
    if state == state_via2pick or state == state_pick2via or
       state == state_via2goal or state == state_goal2via then
        moveIK(position_i, position_f, t_i, t_f)
    end

    if state == state_pick then
        shapeAttached = graspObject()
    end

    if state == state_release then
        releaseObject(shapeAttached)
    end
    
    if state == state_rotate then
        rotateRobot()
    end
end

function moveIK(position_i, position_f, t_i, t_f)
    local x_i, y_i, z_i = unpack(position_i)
    local x_f, y_f, z_f = unpack(position_f)
    local dt = t_f - t_i
    local dx, dy, dz = x_f - x_i, y_f - y_i, z_f - z_i
    local vx, vy, vz = dx / dt, dy / dt, dz / dt
    local x, y, z = x_i + vx * (t - t_i), y_i + vy * (t - t_i), z_i + vz * (t - t_i)

    -- Set the new position for the reference object
    sim.setObjectPosition(RefPosition, -1, {x, y, z})
end

function graspObject()
    local attachedShape
    repeat
        Box = sim.getObjectHandle("Box")
        if Box == -1 then break end

        local result3, distance = sim.checkProximitySensor(Proximity, Box)
        if result3 == 1 then
            attachedShape = Box
            sim.setObjectParent(attachedShape, Connector, true)
            break
        end
    until false
    return attachedShape
end

function releaseObject(shapeAttached)
    sim.setObjectParent(shapeAttached, -1, true)  -- Detach the object
end

function rotateRobot()
    local angle = 0.05

    -- Calculate new orientation based on rotation
    local orientation = sim.getObjectOrientation(RefPosition, -1)
    local newOrientation = {orientation[1], orientation[2], orientation[3] + angle}

    -- Update the orientation of the reference object
    sim.setObjectOrientation(RefPosition, -1, newOrientation)
end