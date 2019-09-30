import UIKit

var str = "Hello, playground"


// Basic Function
func sayHello() {
    print("Hello")
}

sayHello()

// Function with parameters
func sayHelloTo(name:String, age:Int) {
    print("Hello \(name), you are \(age)")
}

sayHelloTo(name: "Tom", age: 20)

// Funciton with return value
func addFourTo(x:Int) -> Int {
    let sum = x + 4
    return sum
}

class Spaceship{
    
    var fuelLevel = 100
    var name = ""
    func cruise() {
        print("Cruising is initiated for \(name)")
    }
    
    func thrust () {
        print("Rocket thrusters intitiated for \(name)")
    }
}

var myShip = Spaceship()
myShip.name = "Tom"
myShip.cruise()

