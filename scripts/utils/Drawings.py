from time import sleep
from world.commons.Draw import Draw


class Drawings():

    def __init__(self,script) -> None:
        self.script = script

    def execute(self):

        # Creating a draw object is done automatically for each agent
        # This is a shortcut to draw shapes without creating an agent
        # Usually, we can access the object through player.world.draw
        a = self.script.args
        draw = Draw(True, 0, a.i, 32769)

        print("\nPress ctrl+c to return.")

        while True:
            for i in range(100):
                sleep(0.02)
            
                draw.circle(    (0,0),i/10,2,      Draw.Color.green_light,"green")
                draw.circle(    (0,0),i/9,2,       Draw.Color.red,"red")
                draw.sphere(    (0,0,5-i/20),0.2,  Draw.Color.red,"ball" )
                draw.annotation((0,0,1), "Hello!", Draw.Color.cyan, "text")
                draw.arrow(     (0,0,5), (0,0,5-i/25), 0.5, 4, Draw.Color.get(127,50,255), "my_arrow")

                #draw pyramid
                draw.polygon(((2,0,0),(3,0,0),(3,1,0),(2,1,0)), Draw.Color.blue, 255, "solid", False)
                draw.line(    (2,0,0), (2.5,0.5,1), 2, Draw.Color.cyan, "solid", False)
                draw.line(    (3,0,0), (2.5,0.5,1), 2, Draw.Color.cyan, "solid", False)
                draw.line(    (2,1,0), (2.5,0.5,1), 2, Draw.Color.cyan, "solid", False)
                draw.line(    (3,1,0), (2.5,0.5,1), 2, Draw.Color.cyan, "solid", True)