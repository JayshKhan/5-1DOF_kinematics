"""
This file contains funny error messages to display
when users input values outside the expected range.

@Author: Jaysh Khan
@response author: Gemini
"""
import random


def get_error_response(min_value, max_value):
    funny_responses = [
        "Whoa there, cowboy! That number is wilder than my weekend plans. Enter a value between [min value] and [max value].",
        "Is that a typo? It looks like your fingers got a little too excited on the keyboard. Pick a number from [min value] to [max value] this time.",
        "Hold on a sec, are you sure you didn't accidentally type in your phone number? We need a value between [min value] and [max value] here.",
        "Nope, that number isn't on the menu today. Try again with a value between [min value] and [max value].",
        "Did you just try to break the system? We only accept numbers between [min value] and [max value]. Don't be a rebel!",
        "That number sounds like it belongs in a galaxy far, far away. We need something a little closer to home - between [min value] and [max value].",
        "Are you using a special measuring system? Ours uses values from [min value] to [max value]. Let's stick to that, shall we?",
        "Maybe you should consult a fortune teller for that number. We only deal in numbers between [min value] and [max value] here.",
        "Looks like you forgot to take your daily dose of common sense. Fear not, a normal number (between [min value] and [max value]) will do just fine.",
        "That number is so out of range, it needs its own passport! Pick a number that can stay within [min value] and [max value].",
        "Whoa, slow down there Speed Racer! We're cruising in the range between [min value] and [max value], not racing off into the sunset.",
        "Did you mistake our input field for a lottery ticket? Keep it grounded within [min value] and [max value] for this ride.",
        "I'm all for exploring new frontiers, but let's keep our exploration within the confines of [min value] and [max value], shall we?",
        "Hmm, I think you're confusing our system with a high-stakes poker game. Stick to the safe bets between [min value] and [max value].",
        "Looks like you're trying to give our system a heart attack with that number. Let's stick to numbers between [min value] and [max value] that won't cause a system meltdown, okay?",
        "I'm afraid that number just took a quantum leap out of our range. Let's bring it back down to Earth, between [min value] and [max value].",
        "Is this a math problem or a riddle? Either way, the solution lies between [min value] and [max value].",
        "If that number were a movie, it'd be rated 'NC-Not in the Calculator'. Stick to numbers within [min value] and [max value] for this feature film.",
        "You're reaching for the stars with that number! Let's keep our feet on the ground and within the range of [min value] and [max value].",
        "I think you just discovered the secret code to unlock the mysteries of the universe! But for this system, let's stick to the tried and true between [min value] and [max value]."
    ]

    return random.choice(funny_responses).replace("[min value]", str(min_value)).replace("[max value]", str(max_value))


def get_error_rresponses_for_singularity():
    funny_responses = [
        "Whoopsie! Your robot seems to be trying out some breakdancing moves. But watch out, it might break itself!",
        "Oh no! Your robot is feeling a bit confused. It's trying to give itself a high-five, but it might end up with a robot slap!",
        "Uh-oh! Your robot is getting adventurous, attempting a game of Twister. But be careful, it might twist itself into a robot knot!",
        "Well, well, well! Your robot seems to be practicing its yoga moves. Downward dog might be fine, but it's best to avoid the downward robot crash!",
        "Hold on a sec! Your robot is feeling a bit rebellious, trying to give itself a pat on the back. But watch out, it might end up with a robot bump!",
        "Oops-a-daisy! Your robot is feeling playful, attempting some cartwheels. But be cautious, it might cartwheel into a robot collision!"
    ]

    return random.choice(funny_responses)
