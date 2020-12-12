--[[
love.physics is a wrapper around the 2D physics engine Box2D
love2d documentation for love.physics objects and functions: https://love2d.org/wiki/love.physics
Box2D Tutorial: http://www.iforce2d.net/b2dtut/introduction

World: love.physics.newWorld(xg, yg, sleep)
    performs physics calculations on all "Bodies" it holds a reference to.
    xg, yg: x and y gravity
    sleep: optional boolean value to allow currently non-moving bodies to sleep (for performance gains).
        Bodies are woken up automatically when something moves close to them. default true.
Body: love.physics.newBody(world, x, y, type)
    does not collide with anything yet. A shape needs to be added to the body with a fixture
    when creating a body, the world gets a reference to it
    by default, the body position (x, y) defines the center of the body and not the top left
    local coordinates are relative to the body position, world coordinates are relative to (0, 0)
    has the following properties:
        mass: getMass()
        velocity: getLinearVelocity()
        rotational inertia: getInertia()
        angular velocity: getAngularVelocity()
        position: getPosition()
        angle: getAngle()
    the usual way of giving mass to the body is by adding fixtures to it
    3 body types:
        dynamic: full physical behavior. affected by gravity, forces and collisions with other objects
        static: cannot move (although setting the position directly is possible)
        kinematic: like static, but can move and rotate. Is not influenced by collisions/ forces/ gravity.
Fixture: love.physics.newFixture(body, shape, density)
    a fixture adds a shape to the body to give it physical characteristics.
    multiple fixtures can be attached to a body
    has the following properties:
        restitution: getRestitution(). Bounciness.
            ratio of the total velocity of both fixtures before : after collision
            the greater Restitution of both fixtures applies
            affects the velocity component normal (perpendicular) to the surface
        friction: getFriction(). Slipperiness.
            value between 0 and 1. The total friction between 2 fixtures is: sqrt(friction1 * friction2)
            affects the velocity component tangential (parallel) to the surface
        density: getDensity(). defines the weight. default 1 kg/m^2
    a fixture can be a sensor by using setSensor().
    A Sensor does not collide but collision callback functions are called like normal.
Shape:
    shape coordinates are relative to its body coordinates.
    If for a polygon shape the center of mass should coincide with the body position,
    its vertices should be specified relative to the center of the shape (for a body with 1 shape)
    5 different shapes:
        circle: love.physics.newCircleShape(radius). centered by default
        rectangle: love.physics.newRectangleShape(width, height). centered by default
        edge: love.physics.newEdgeShape(x, y, width, height). line segment
        chain: love.physics.newChainShape(loop, x1, y1, x2, y2, ...). multiple line segments.
            can be used to create terrain boundaries
        polygon: love.physics.newPolygonShape(x1, y1, x2, y2, x3, y3, ...). 3 <= vertices <= 8.
further topics: collision callbacks, collision filtering, fixture user data, joints, ray casting, ...
]]
--[[
    Ball pit to demonstrate Box2D
]]

VIRTUAL_WIDTH = 1920
VIRTUAL_HEIGHT = 1080

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

local push = require 'push'

-- mouse position
local mouse_x, mouse_y = 0, 0
-- Box2D world object
local world = nil
-- body that should follow the mouse movement
local selected_body = nil
-- body for the ground
local ground_body = nil
-- all kinematic or dynamic bodies of the world
local kinematic_bodies = {}
local dynamic_bodies = {}
local dynamic_bodies_colors = {}
local weighted_sq_body = nil

PIXEL_PER_METER = 80                -- definition of 1 meter
GRAVITY = 9.81 * PIXEL_PER_METER    -- in pixel / s^2

function love.load()
    math.randomseed(os.time())
    love.graphics.setDefaultFilter('linear', 'linear')
    love.window.setTitle('Box2D Demonstration')

    push:setupScreen(VIRTUAL_WIDTH, VIRTUAL_HEIGHT, WINDOW_WIDTH, WINDOW_HEIGHT, {
        fullscreen = false,
        vsync = true,
        resizable = true
    })

    -- set pixel / meter (default 30)
    -- Box2D is tuned to work well with shape sizes from 0.1 to 10 meters
    love.physics.setMeter(PIXEL_PER_METER)

    -- new Box2D "world" which will run all physics calculations
    world = love.physics.newWorld(0, GRAVITY)

    -- static ground and wall bodies
    local ground_height = 30
    ground_body = love.physics.newBody(world, 0, VIRTUAL_HEIGHT - ground_height, 'static')
    local left_wall_body = love.physics.newBody(world, 0, 0, 'static')
    local right_wall_body = love.physics.newBody(world, VIRTUAL_WIDTH, 0, 'static')
    local ceiling_body = love.physics.newBody(world, 0, 0, 'static')
    -- the edge shape is suitable for ground and walls
    local wall_shape_hor = love.physics.newEdgeShape(0, 0, VIRTUAL_WIDTH, 0)
    local wall_shape_vert = love.physics.newEdgeShape(0, 0, 0, VIRTUAL_HEIGHT)
    -- create a fixture that attaches a shape to the body
    local ground_fixture = love.physics.newFixture(ground_body, wall_shape_hor)
    local left_wall_fixture = love.physics.newFixture(left_wall_body, wall_shape_vert)
    local right_wall_fixture = love.physics.newFixture(right_wall_body, wall_shape_vert)
    local ceiling_fixture = love.physics.newFixture(ceiling_body, wall_shape_hor)
    -- set restitution so the bodies bounce a bit off of the wall
    local wall_restitution = 0.05
    ground_fixture:setRestitution(wall_restitution)
    left_wall_fixture:setRestitution(wall_restitution)
    right_wall_fixture:setRestitution(wall_restitution)
    ceiling_fixture:setRestitution(wall_restitution)

    -- create multiple kinematic bodies with a polygon shape that will rotate
    kinematic_bodies = {}
    local kinematic_body_size = 60          -- edge length of the polygon
    local kinematic_body_distance = 150     -- x distance between the bodies
    -- spawn point of the first body inside the world
    local kinematic_bodies_start_x, kinematic_bodies_start_y = 100, VIRTUAL_HEIGHT / 2
    -- shape vertices. make an equilateral triangle.
    local kinematic_shape_points = {
        0, 0,
        kinematic_body_size, 0,
        kinematic_body_size / 2, math.sqrt(kinematic_body_size^2 - (kinematic_body_size / 2)^2)
    }
    -- make shape coordinates relative to body center.
    -- This is needed because a kinematic body does not have a center of mass (because it doesn't have mass)
    -- So the rotation for a kinematic body is applied about the body position
    -- get the center of mass for the shape (for a supposed density of 1 kg/m^2)
    local kin_center_x, kin_center_y = love.physics.newPolygonShape(kinematic_shape_points):computeMass(1)
    for i = 1, #kinematic_shape_points do
        -- table elements alternate between x and y coordinates
        kinematic_shape_points[i] = i % 2 == 1
            and kinematic_shape_points[i] - kin_center_x or kinematic_shape_points[i] - kin_center_y
    end
    local kinematic_shape = love.physics.newPolygonShape(kinematic_shape_points)
    for i = 1, 3 do
        table.insert(kinematic_bodies, love.physics.newBody(world,
            kinematic_bodies_start_x + kinematic_body_distance * (i - 1), kinematic_bodies_start_y, 'kinematic'))
        love.physics.newFixture(kinematic_bodies[i], kinematic_shape)
        -- set rotation in rad/s
        kinematic_bodies[i]:setAngularVelocity(math.rad(180))
    end

    -- heavy square body
    local weighted_sq_size = 100
    local weighted_sq_density = 10
    weighted_sq_body = love.physics.newBody(world,
        math.random(weighted_sq_size / 2, VIRTUAL_WIDTH - weighted_sq_size / 2), weighted_sq_size / 2, 'dynamic')
    local weighted_sq_shape = love.physics.newRectangleShape(weighted_sq_size, weighted_sq_size)
    love.physics.newFixture(weighted_sq_body, weighted_sq_shape, weighted_sq_density)

    -- dynamic bodies (balls)
    dynamic_bodies = {}
    dynamic_bodies_colors = {}      -- random color for each ball
    local ball_size = 15            -- radius
    local ball_shape = love.physics.newCircleShape(ball_size)
    for i = 1, 1000 do
        table.insert(dynamic_bodies,
            love.physics.newBody(world,
                math.random(ball_size / 2, VIRTUAL_WIDTH - ball_size / 2),
                math.random(ball_size / 2, VIRTUAL_HEIGHT - ground_height - ball_size / 2), 'dynamic')
        )
        table.insert(dynamic_bodies_colors, {
            r = math.random(255),
            g = math.random(255),
            b = math.random(255)
        })
        love.physics.newFixture(dynamic_bodies[i], ball_shape)
    end
end

-- callback function that is called when a world bounding box query is conducted with queryBoundingBox()
-- the function is called for every fixture found in the query
-- return: boolean value. if true, continue calling the function for for the found fixtures
-- if false, stop calling this function for the query.
function callbackWorldQuery(fixture)
    -- determine the body that should follow the mouse.
    -- only select the body if its dynamic and the mouse is inside its fixture
    -- stop searching when a suitable body was found
    selected_body = fixture:getBody()
    mouse_x, mouse_y = push:toGame(love.mouse.getPosition())

    if not fixture:testPoint(mouse_x, mouse_y) or selected_body:getType() ~= 'dynamic' then
        selected_body = nil
        return true
    end

    return false
end

function love.resize(w, h)
    push:resize(w, h)
end

function love.keypressed(key)
    if love.keyboard.isDown('lalt') and key == 'return' then
        push:switchFullscreen()
        return
    end
    if key == 'escape' then
        love.event.quit()
    end
end

function love.mousepressed(x, y, button)
    x, y = push:toGame(x, y)
    -- if pressed left mouse button, search for fixtures in the range of the mouse.
    -- the bounding box to search in is just the point where the mouse is in this case.
    -- for every fixture with an axis aligned bounding box (AABB) that intersects the search box
    -- the specified callback function is called immediately after.
    if button == 1 then
        world:queryBoundingBox(x, y, x, y, callbackWorldQuery)
    -- if pressed right mouse button, teleport the body to the mouse pointer
    elseif button == 2 then
        weighted_sq_body:setPosition(x, y)
        weighted_sq_body:setLinearVelocity(0, 0)
    end
end

function love.mousereleased(x, y, button)
    if button == 1 then
        selected_body = nil
    end
end

function love.update(dt)
    -- limit dt
    dt = math.min(dt, 0.07)

    -- control a dynamic body that was selected by holding down the left mouse button
    -- apply forces to the body to reach the mouse position (target point)
    -- applying impulses or forces to the body is the preferred solution
    -- setting position or velocity directly is suboptimal and leads to non-physical behavior
    mouse_x, mouse_y = love.mouse.getPosition()
    -- convert to virtual resolution coordinates (can return nil when resizing the window)
    mouse_x, mouse_y = push:toGame(mouse_x, mouse_y)
    if mouse_x and mouse_y and selected_body then
        -- target velocity and maximum velocity that the body should have in order to reach its target point
        local target_vel = 1000     -- in pixel / s

        -- divide the force to apply to the body, so it doesn't reach the target velocity immediately.
        -- if set too high, the force that is applied is not sufficient to move the body to the target
        -- (when applied force is lower than the gravitational force) and the overshooting will be high.
        -- if set too low, the applied force will be very high at the start and end of movement,
        -- which could introduce some bugs.
        -- alternative implementation: limit the allowed change in velocity per frame
        local force_div = 3

        -- the future position of the body is predicted for this number of frames (time steps).
        -- the prediction is made under the assumption of constant velocity over this number of frames.
        -- the prediction is used to determine if the target point will be reached in this number of frames.
        -- if yes, target_vel shall be reduced. This needs to be a value of 1 or higher, otherwise the body will
        -- revolve around the target indefinitely (because it will overshoot the target every time unattenuated).
        -- the higher the number, the earlier the body will decelerate before reaching the target point.
        -- low values will produce a slow convergence to the target point and have higher overshooting.
        local num_frames_foresight = 3

        -- get the center of mass of the body and calculate the distance to the target point
        local body_x, body_y = selected_body:getPosition()
        local dist_to_target_x = mouse_x - body_x
        local dist_to_target_y = mouse_y - body_y
        local dist_to_target = math.sqrt(dist_to_target_x^2 + dist_to_target_y^2)
        -- calculate the target velocity vector (points to the target and has the amount of target_vel)
        local target_vel_x = 0
        local target_vel_y = 0
        if dist_to_target ~= 0 then     -- don't divide by 0
            target_vel_x = (dist_to_target_x / dist_to_target) * target_vel
            target_vel_y = (dist_to_target_y / dist_to_target) * target_vel
        end
        -- set the target velocity to the value needed to hit the target exactly
        -- in the amount of time steps specified by num_frames_foresight,
        -- if the body will overshoot its target in this amount of time steps.
        local next_pos_x = target_vel_x * dt * num_frames_foresight
        local next_pos_y = target_vel_y * dt * num_frames_foresight
        local next_pos = math.sqrt(next_pos_x^2 + next_pos_y^2)
        if dist_to_target < next_pos and next_pos ~= 0 then
            target_vel_x = target_vel_x * (dist_to_target / next_pos)
            target_vel_y = target_vel_y * (dist_to_target / next_pos)
        end

        -- calculate the force from the target velocity
        -- F = m * a
        -- a = delta_v / delta_t
        local body_v_x, body_v_y = selected_body:getLinearVelocity()
        local dv_x = target_vel_x - body_v_x
        local dv_y = target_vel_y - body_v_y
        local force_x = selected_body:getMass() * dv_x / dt
        local force_y = selected_body:getMass() * dv_y / dt
        force_x = force_x / force_div
        force_y = force_y / force_div
        selected_body:applyForce(force_x, force_y);
    end

    -- normalize angles of rotating bodies (otherwise the angle will grow indefinitely)
    for i = 1, #kinematic_bodies do
        kinematic_bodies[i]:setAngle(kinematic_bodies[i]:getAngle() % (2 * math.pi))
    end

    -- update world, calculating collisions
    -- The game runs at 60 fps. The time step for the Box2D Simulation will therefore be 1/60 seconds
    world:update(dt)
end

function love.draw()
    push:start()

    -- draw a line that represents the ground, calculated from ground body and shape
    love.graphics.setColor(255/255, 255/255, 255/255, 255/255)
    love.graphics.setLineWidth(10)
    local ground_shape = ground_body:getFixtures()[1]:getShape()
    -- Get the local coordinates of the polygon's vertices and transform them into world coordinates.
    -- For the ground body this will result in x1, y1, x2, y2 which can be used as input to love.graphics.line()
    love.graphics.line(ground_body:getWorldPoints(ground_shape:getPoints()))

    -- draw kinematic bodies. The vertices fit as an input to love.graphics.polygon()
    love.graphics.setColor(0/255, 0/255, 255/255, 255/255)
    local kinematic_shape = kinematic_bodies[1]:getFixtures()[1]:getShape()
    for i = 1, #kinematic_bodies do
        love.graphics.polygon('fill', kinematic_bodies[i]:getWorldPoints(kinematic_shape:getPoints()))
    end

    -- render balls in the ball pit
    local ball_shape = dynamic_bodies[1]:getFixtures()[1]:getShape()
    for i = 1, #dynamic_bodies do
        love.graphics.setColor(
            dynamic_bodies_colors[i].r/255, dynamic_bodies_colors[i].g/255, dynamic_bodies_colors[i].b/255, 255/255)
        love.graphics.circle('fill', dynamic_bodies[i]:getX(), dynamic_bodies[i]:getY(), ball_shape:getRadius())
    end

    -- render the heavy square body
    love.graphics.setColor(255/255, 255/255, 255/255, 255/255)
    local weighted_sq_shape = weighted_sq_body:getFixtures()[1]:getShape()
    love.graphics.polygon('fill', weighted_sq_body:getWorldPoints(weighted_sq_shape:getPoints()))

    push:finish()
end
