let Prelude = ./Prelude.dhall

let Map = Prelude.Map

let builder =
      \(a : Type) ->
      \(b : Type) ->
      \(key : a) ->
      \(value : b) ->
        { mapKey = key, mapValue = value }

let EnumeratedValue =
      < Automatic : List Text | Explicit : Map.Type Text Integer >

let SignalName = Text

let Signal =
      { Type =
          { description : Text
          , width : Natural
          , enumerated_values : Optional EnumeratedValue
          , scale : Optional Double
          , twos_complement : Optional Bool
          }
      , default =
        { enumerated_values = None EnumeratedValue
        , scale = None Double
        , twos_complement = None Bool
        }
      }

let signal = builder SignalName Signal.Type

let MessageName = Text

let Message =
      { Type =
          { id : Natural
          , cycletime : Natural
          , signals : Optional (Map.Type SignalName Signal.Type)
          }
      , default.signals = None (Map.Type SignalName Signal.Type)
      }

let message = builder MessageName Message.Type

let NodeName = Text

let Node =
      { Type =
          { rx : Optional (List MessageName)
          , messages : Optional (Map.Type MessageName Message.Type)
          }
      , default =
        { rx = None (List MessageName)
        , messages = None (Map.Type MessageName Message.Type)
        }
      }

let node = builder NodeName Node.Type

let NetworkName = Text

let Network = { bitrate : Natural, nodes : Map.Type NodeName Node.Type }

let network = builder NetworkName Network

let Config =
      { Type = { networks : Map.Type NetworkName Network }, default = {=} }

in  { EnumeratedValue
    , SignalName
    , Signal
    , signal
    , MessageName
    , Message
    , message
    , NodeName
    , Node
    , node
    , NetworkName
    , Network
    , network
    , Config
    }
