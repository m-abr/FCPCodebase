from agent.Base_Agent import Base_Agent as Agent
from itertools import count
from scripts.commons.Script import Script

'''
How does communication work?
    The say command allows a player to broadcast a message to everyone on the field
    Message range: 50m (the field is 36m diagonally, so ignore this limitation)
    The hear perceptor indicates 3 things: 
        - the message
        - the origin team
        - the origin absolute angle (set to "self" if the message was sent by oneself)

    Messages are heard in the next step.
    Messages are only sent every 2 steps (0.04s).
    Messages sent in muted steps are only heard by oneself.
    In one time step, a player can only hear one other player besides itself.
    If two other players say something, only the first message is heard.
    This ability exists independetly for messages from both teams.
    In theory, a player can hear its own message + the 1st teammate to speak + the 1st opponent to speak
    In practice, the opponent doesn't matter because our team's parser ignores messages from other teams

    Message characteristics:
        Maximum 20 characters, ascii between 0x20, 0x7E except ' ', '(', ')'
        Accepted: letters+numbers+symbols: !"#$%&'*+,-./:;<=>?@[\]^_`{|}~
        However, due to a server bug, sending ' or " ends the message sooner
            
'''

class Team_Communication():

    def __init__(self,script:Script) -> None:
        self.script = script

    def player1_hear(self, msg:bytes, direction, timestamp:float) -> None:
        print(f"Player 1 heard: {msg.decode():20}  from:{direction:7}  timestamp:{timestamp}")

    def player2_hear(self, msg:bytes, direction, timestamp:float) -> None:
        print(f"Player 2 heard: {msg.decode():20}  from:{direction:7}  timestamp:{timestamp}")

    def player3_hear(self, msg:bytes, direction, timestamp:float) -> None:
        print(f"Player 3 heard: {msg.decode():20}  from:{direction:7}  timestamp:{timestamp}")
     
        
    def execute(self):

        a = self.script.args

        hear_callbacks = (self.player1_hear, self.player2_hear, self.player3_hear)

        # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name, Enable Log, Enable Draw, Play Mode Correction, Wait for Server, Hear Callback
        self.script.batch_create(Agent, ((a.i,a.p,a.m,i+1,0,a.t,True,True,False,True,clbk) for i,clbk in enumerate(hear_callbacks)))
        p1:Agent = self.script.players[0]
        p2:Agent = self.script.players[1]
        p3:Agent = self.script.players[2]

        # Beam players
        self.script.batch_commit_beam( [(-2,i,45) for i in range(3)] )

        for i in count():
            msg1 = b"I_am_p1!_no:"+str(i).encode()
            msg2 = b"I_am_p2!_no:"+str(i).encode()
            msg3 = b"I_am_p3!_no:"+str(i).encode()
            p1.scom.commit_announcement(msg1)                            # commit message
            p2.scom.commit_announcement(msg2)                            # commit message
            p3.scom.commit_announcement(msg3)                            # commit message
            self.script.batch_commit_and_send()              # send message
            print(f"Player 1 sent:  {msg1.decode()}      HEX: {' '.join([f'{m:02X}' for m in msg1])}")
            print(f"Player 2 sent:  {msg2.decode()}      HEX: {' '.join([f'{m:02X}' for m in msg2])}")
            print(f"Player 3 sent:  {msg3.decode()}      HEX: {' '.join([f'{m:02X}' for m in msg3])}")
            self.script.batch_receive()
            input("Press enter to continue or ctrl+c to return.")

