//
//  ViewController.swift
//  WarCardGame
//
//  Created by Neil Mathew on 8/26/19.
//  Copyright © 2019 Neil Mathew. All rights reserved.
//

import UIKit

class ViewController: UIViewController {
    
    
    @IBOutlet weak var leftimageview: UIImageView!
    
    @IBOutlet weak var Rightimageview: UIImageView!
    
    @IBOutlet weak var leftScoreLabel: UILabel!
    
    @IBOutlet weak var rightScoreLabel: UILabel!
    
    var leftScore = 0
    var rightScore = 0
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
        
        
    }
    
    @IBAction func dealTapped(_ sender: Any) {
        // Randomize numbers
        let leftNumber = Int.random(in: 2...14)
        let rightNumber = Int.random(in: 2...14)
        
        // Update the image views
        leftimageview.image = UIImage(named: "card\(leftNumber)")
        Rightimageview.image = UIImage(named: "card\(rightNumber)")
        
        // Compare the random numbers
        if leftNumber > rightNumber {
            leftScore += 1
            leftScoreLabel.text = String(leftScore)
            
        }
        else if rightNumber > leftNumber {
            rightScore += 1
            rightScoreLabel.text = String(rightScore)
        }
        else {
            
        }
    }
    


}

