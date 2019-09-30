//
//  theApp.swift
//  ScoutingApp
//
//  Created by Neil Mathew on 9/14/19.
//  Copyright © 2019 Neil Mathew. All rights reserved.
//

import Foundation
import Firebase

public class theApp {
    public static let sharedInstance = theApp()
    
    // Constants for this App
    let APPNAME = "Neil's App"
    private var m_ModelInitialized : Bool = false
    private var m_AppInitialized : Bool = false
    private var loggedIn = false
    
    // Set the firebase reference
    let fbref = Database.database().reference()
    
}
