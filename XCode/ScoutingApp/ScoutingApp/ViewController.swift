//
//  ViewController.swift
//  ScoutingApp
//
//  Created by Neil Mathew on 8/28/19.
//  Copyright © 2019 Neil Mathew. All rights reserved.
//

import UIKit
import FirebaseDatabase

class ViewController: UIViewController, UIPickerViewDataSource, UIPickerViewDelegate {
    
    // Connect objects to view controller
    @IBOutlet weak var eventPickerView: UIPickerView!
    
    var ref:DatabaseReference?
    var databaseHandle:DatabaseHandle?
    
    // Define variables
    var glibs = [String]()
    
    // Use functions necessary for pickerview
    func numberOfComponents(in pickerView: UIPickerView) -> Int {
        return 1
    }
    func pickerView(_ pickerView: UIPickerView, titleForRow row: Int, forComponent component: Int) -> String? {
        return glibs[row]
    }
    func pickerView(_ pickerView: UIPickerView, numberOfRowsInComponent component: Int) -> Int {
        return glibs.count
    }
    
    // The normal XCode stuff
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
        
        
        // Retrieve the posts and listen for any changes to the Events branch
        databaseHandle = theApp.sharedInstance.fbref.child("Events").observe(.childAdded, with: { (snapshot) in
            
            let eventName = snapshot.childSnapshot(forPath: "EventName").value as? String
            if let eventName = eventName {
                self.glibs.append(eventName)
            }
            
        })
        
    }
    
    
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
    }


}

