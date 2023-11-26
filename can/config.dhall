let types = ./types.dhall

in  types.Config::{
    , networks =
      [ types.network
          "PARTY"
          { bitrate = 500000
          , nodes =
            [ types.node
                "BLINK"
                types.Node::{
                , messages = Some
                  [ types.message
                      "Test"
                      types.Message::{
                      , id = 0x345
                      , cycletime = 100
                      , signals = Some
                        [ types.signal
                            "blinkState"
                            types.Signal::{ description = "foo", width = 1 }
                        ]
                      }
                  ]
                }
            ]
          }
      ]
    }
