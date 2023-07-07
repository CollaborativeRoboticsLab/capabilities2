# an abstract robot capability
# the capability is described in a form that can be encoded or translated into natural language and back
capabilities2_msgs/Capability capability
string description
string type
capabilities2_msgs/CapabilityCommand command

###
# standard abstract robot capabilities
###

# general
string GENERAL = "general"
string ACTION = "action"
string BEHAVIOUR = "behaviour"
string TASK = "task"
string ROLE = "role"

# state flow
string STOP = "stop"
string START = "start"
string PAUSE = "pause"
string RESUME = "resume"
string RESTART = "restart"
string CANCEL = "cancel"
string ENABLE = "enable"
string DISABLE = "disable"

# motion
string MOVE = "move"
string TELEPORT = "teleport"
string TURN = "turn"
string ROTATE = "rotate"
string TRANSLATE = "translate"
string RUN = "run"
string WALK = "walk"
string JUMP = "jump"
string FLY = "fly"
string SWIM = "swim"
string CLIMB = "climb"
string CRAWL = "crawl"
string DIG = "dig"

# operation
string OPERATE = "operate"
string USE = "use"
string INTERACT = "interact"
string BUILD = "build"
string APPROACH = "approach"
string RETRIEVE = "retrieve"
string STORE = "store"
string SEARCH = "search"

# navigation
string NAVIGATE = "navigate"
string EXPLORE = "explore"
string LOCALIZE = "localize"
string MAP = "map"
string PATH = "path"
string GOAL = "goal"
string WAYPOINT = "waypoint"
string OBSTACLE = "obstacle"
string AVOID = "avoid"

# general interaction contexts
string INTERACT_SELF = "interact_self"
string INTERACT_OTHER = "interact_other"
string INTERACT_OBJECT = "interact_object"
string INTERACT_ENVIRONMENT = "interact_environment"
string INTERACT_HUMAN = "interact_human"
string INTERACT_ANIMAL = "interact_animal"
string INTERACT_LIVING = "interact_living"
string INTERACT_PLANT = "interact_plant"
string INTERACT_NONLIVING = "interact_nonliving"

# general interaction actions
string CONSTRUCT = "construct"
string DESTRUCT = "destruct"
string CONSUME = "consume"
string PRODUCE = "produce"
string REPAIR = "repair"
string DESTROY = "destroy"
string CREATE = "create"
string MODIFY = "modify"
string ACTIVATE = "activate"
string DEACTIVATE = "deactivate"

# access
string OPEN = "open"
string CLOSE = "close"
string LOCK = "lock"
string UNLOCK = "unlock"

# manipulation
string PICKUP = "pickup"
string DROP = "drop"
string GRAB = "grab"
string RELEASE = "release"
string PUSH = "push"
string PULL = "pull"
string INTERACT_TURN = "interact_turn"
string INTERACT_ROTATE = "interact_rotate"
string INTERACT_TRANSLATE = "interact_translate"
string INTERACT_SCALE = "interact_scale"

# file operations
string COPY = "copy"
string PASTE = "paste"
string CUT = "cut"
string PRINT = "print"
string READ = "read"
string WRITE = "write"

# communication
string SPEAK = "speak"
string LISTEN = "listen"

# perception
string SEE = "see"
string HEAR = "hear"
string SMELL = "smell"
string TASTE = "taste"
string TOUCH = "touch"

# cognition
string THINK = "think"
string REMEMBER = "remember"
string LEARN = "learn"
string UNDERSTAND = "understand"
string REASON = "reason"
string DECIDE = "decide"
string PLAN = "plan"
string IMAGINE = "imagine"
string BELIEVE = "believe"
string KNOW = "know"
string PERCEIVE = "perceive"

# social
string FEEL = "feel"
string EMOTE = "emote"
string GESTURE = "gesture"
string DANCE = "dance"
string SING = "sing"
string PLAY = "play"
string FIGHT = "fight"
string ATTACK = "attack"
string DEFEND = "defend"
