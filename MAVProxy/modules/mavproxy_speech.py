#!/usr/bin/env python
'''

Speech Engine for audio announcements of MAVLink messages during flight.

Suggested usage: Add initialization commands to .mavinit.scr

Example commands:
    module load speech
    speech add VFR_HUD.airspeed onchange every=3.0 as=airspeed format=%0.0f threshold=2.0
    speech add ALTITUDE.altitude_relative onchange every=3.0 as=altitude format=%0.0f threshold=5.0

'''

import time, os, math, traceback
from MAVProxy.modules.lib import mp_module

class PeriodicSpeechAnnounce(object):
    '''
    SpeechAnnouncement objects encapsulate all functionality and responsibility
    of generating speech for a single mavlink message entry.

    The default SpeechAnnounce module will provide mavlink-to-text and rate-limiting
    capabilities.
    '''

    def __init__(self, module, msg_type, entry_name, sayEveryS=30.0, sayAs=None, scale=1, formatstr="%0.0f", voice=None):
        self.module = module
        self.msg_type = msg_type
        self.entry_name = entry_name
        self.sayEveryS = sayEveryS
        self.sayAs = sayAs
        self.scale = scale
        self.formatstr = formatstr
        self.voice = voice
        self.priority = 'normal'

        self.lastTime = 0

    def set_every(self, every):
        self.sayEveryS = every

    def set_as(self, say_as):
        self.sayAs = say_as

    def set_scale(self, scale):
        self.scale = scale

    def set_format(self, formatstr):
        self.formatstr = formatstr

    def set_voice(self, voice):
        self.voice = voice

    def set_priority(self, priority):
        self.priority = priority

    def match(self, msg):
        '''Returns true if this announce is ready to handle the given message'''
        return (msg.get_type() == self.msg_type)

    def dispatch(self, msg, backend):
        '''Dispatch this announcement IF it can handle the given message'''
        if self.match(msg):
            self.maybe_say(msg, backend)

    def msg_entry_to_text(self, msg):
        return getattr(msg, self.entry_name, 0)

    def maybe_say(self, msg, backend, force=False):
        '''Calls backend with this announce for the given message IF the announce passes internal checks'''
        now = time.time()
        if force or (now - self.lastTime > self.sayEveryS):
            self.lastTime = now
            value = self.msg_entry_to_text(msg)
            backend(self.to_speech_text(value), priority=self.priority, voice=self.voice)
            return True
        return False

    def to_speech_text(self, value):
        '''Returns the text announced before the value'''
        return "%s %s" % (self.get_speech_name(), self.convert(value))

    def get_speech_name(self):
        if self.sayAs:
            return self.sayAs
        else:
            return ""

    def convert(self, value):
        '''Returns the announce text representation of the value'''
        return self.formatstr % value #(value*self.scale)

    def __str__(self):
        voice_str = "default"
        if (self.voice):
            voice_str = self.voice
        return "%s.%s as \"%s\" every %0.0f seconds, scaling by %0.4f and formatting as %s with %s voice" % (
            self.msg_type, 
            self.entry_name, 
            self.get_speech_name(), 
            self.sayEveryS,
            self.scale,
            self.formatstr,
            voice_str)

class ConditionalSpeechAnnounce(PeriodicSpeechAnnounce):
    '''
    A SpeechAnnounce subclass that provides rate-limited announcements
    only when a mavlink value is outside an expected range.
    '''

    def __init__(self, module, msg_type, entry_name, sayEveryS=5.0, sayAs=None, scale=1, formatstr="%0.0f", voice=None, range=[-float("inf"), float("inf")]):
        super(ConditionalSpeechAnnounce, self).__init__(module, msg_type, entry_name, sayEveryS=sayEveryS, sayAs=sayAs, scale=scale, formatstr=formatstr, voice=None)
        self.range = range
        self.set_voice("Daniel")

    def set_min(self, minval):
        self.range[1] = minval

    def set_max(self, maxval):
        self.range[1] = maxval

    def maybe_say(self, msg, backend, force=False):
        value = self.msg_entry_to_text(msg)
        if force or value < self.range[0] or value > self.range[1]:
            return super(ConditionalSpeechAnnounce, self).maybe_say(msg, backend, force=force)
        return False

    def to_speech_text(self, value):
        if value < self.range[0]:
            return "%s below minimum! Expected minimum %s, current value %s." % (
                self.get_speech_name(), self.convert(self.range[0]), self.convert(value))
        if value > self.range[1]:
            return "%s above maximum! Expected maximum %s, current value %s." % (
                self.get_speech_name(), self.convert(self.range[1]), self.convert(value))

    def __str__(self):
        s = super(ConditionalSpeechAnnounce, self).__str__()
        return "%s when value is outside [%f, %f]" % (s, self.range[0], self.range[1])  

class OnChangeSpeechAnnounce(PeriodicSpeechAnnounce):
    '''
    A SpeechAnnounce subclass that provides rate-limited announcements
    only when a mavlink value changes.
    '''

    def __init__(self, module, msg_type, entry_name, sayEveryS=5.0, sayAs=None, scale=1, formatstr="%0.0f", voice=None, threshold=0):
        super(OnChangeSpeechAnnounce, self).__init__(module, msg_type, entry_name, sayEveryS=sayEveryS, sayAs=sayAs, scale=scale, formatstr=formatstr, voice=None)
        self.prev_value = float("inf")
        self.threshold = threshold
        self.set_voice("Daniel")
        self.set_priority('important')
        
    def set_threshold(self, threshold):
        self.threshold = threshold

    def maybe_say(self, msg, backend, force=False):
        new_value = self.msg_entry_to_text(msg)
        if self.prev_value == float("inf"):
            self.prev_value = new_value
        if force or (self.threshold == 0 and self.prev_value != new_value) or (self.threshold > 0 and abs(self.prev_value - new_value) >= self.threshold):
            self.prev_value = new_value
            super(OnChangeSpeechAnnounce, self).maybe_say(msg, backend, force=force)

    def __str__(self):
        s = super(OnChangeSpeechAnnounce, self).__str__()
        return "%s when value changes by more than %f" % (s, self.threshold)  

class PeriodicOnChangeSpeechAnnounce(PeriodicSpeechAnnounce):
    '''
    A SpeechAnnounce subclass that provides rate-limited announcements
    only when a mavlink value changes.
    '''

    def __init__(self, module, msg_type, entry_name, sayEveryS=5.0, sayAs=None, scale=1, formatstr="%0.0f", voice=None, threshold=0):
        super(PeriodicOnChangeSpeechAnnounce, self).__init__(module, msg_type, entry_name, sayEveryS=sayEveryS, sayAs=sayAs, scale=scale, formatstr=formatstr, voice=None)
        self.prev_value = float("inf")
        self.threshold = threshold
        self.set_voice(None)
        self.ratelimit = 0

    def set_threshold(self, threshold):
        self.threshold = threshold

    def set_ratelimit(self, ratelimit):
        self.ratelimit = ratelimit

    def maybe_say(self, msg, backend, force=False):
        new_value = self.msg_entry_to_text(msg)
        if self.prev_value == float("inf"):
            self.prev_value = new_value

        now = time.time()
        if not force and (now - self.lastTime < self.ratelimit):
            return

        if not(super(PeriodicOnChangeSpeechAnnounce, self).maybe_say(msg, backend, force)):
            if (self.threshold == 0 and self.prev_value != new_value) or (self.threshold > 0 and abs(self.prev_value - new_value) >= self.threshold):
                self.prev_value = new_value
                super(PeriodicOnChangeSpeechAnnounce, self).maybe_say(msg, backend, force=True)

        else:
            self.prev_value = new_value

    def __str__(self):
        s = super(PeriodicOnChangeSpeechAnnounce, self).__str__()
        return "%s periodically AND when value changes by more than %f (rate-limited to %fs)" % (s, self.threshold, self.ratelimit)  


import threading
import Queue
import subprocess

class SaySpeechBackend(threading.Thread):
    '''
    Thread responsible for managing macOS "say" command line process.
    Provides a nonblocking but serialized interface to "say".
    '''

    def __init__(self):
        threading.Thread.__init__(self)
        self.to_say_queue = Queue.Queue()
        self.daemon = True

    def say(self, text, priority='normal', voice=None):
        if (self.to_say_queue.qsize() > 2):
            print "FLUSHING TO_SAY_QUEUE. Size is ", self.to_say_queue.qsize()
            with self.to_say_queue.mutex:
                self.to_say_queue.queue.clear()
        if (priority == 'important'):
            with self.to_say_queue.mutex:
                self.to_say_queue.queue.clear()
        self.to_say_queue.put((text, priority, voice))

    def run(self):
        '''thread loop here'''
        while True:
            try:
                # We loop once a second and give MAVProxy a chance to kill threads.
                (text, priority, voice) = self.to_say_queue.get(True, 1.0)
                if (voice):
                    subprocess.call(['say', '-v', voice, text])
                else:
                    subprocess.call(['say', text])
                self.to_say_queue.task_done()
            except Queue.Empty:
                pass

class SpeechModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SpeechModule, self).__init__(mpstate, "speech", "speech output")
        self.add_command('speech', self.cmd_speech, "text-to-speech", ['<test>'])

        self.msgs_to_announce = []

        self.old_mpstate_say_function = self.mpstate.functions.say
        self.mpstate.functions.say = self.print_and_say
        self.enabled_for = "ALL"
        self.say_speech_backend = None

        try:
            self.settings.get('speech')
        except AttributeError:
            self.settings.append(('speech', int, 1))
        try:
            self.settings.set('speech_voice', '')
        except AttributeError:
            self.settings.append(('speech_voice', str, ''))
        self.kill_speech_dispatcher()
        for (backend_name,backend) in [("speechd",self.say_using_speechd), ("espeak",self.say_using_espeak), ("speech", self.say_using_speech), ("say", self.say_using_say)]:
            try:
                backend("")
                self.say_backend = backend
                print("Using speech backend '%s'" % backend_name)
                return
            except Exception:
                pass
        self.say_backend = None
        print("No speech available")

    def kill_speech_dispatcher(self):
        '''kill speech dispatcher processs'''
        if not 'HOME' in os.environ:
            return
        pidpath = os.path.join(os.environ['HOME'], '.speech-dispatcher',
                               'pid', 'speech-dispatcher.pid')
        if os.path.exists(pidpath):
            try:
                import signal
                pid = int(open(pidpath).read())
                if pid > 1 and os.kill(pid, 0) is None:
                    print("Killing speech dispatcher pid %u" % pid)
                    os.kill(pid, signal.SIGINT)
                    time.sleep(1)
            except Exception as e:
                pass


    def unload(self):
        '''unload module'''
        self.settings.set('speech', 0)
        if self.mpstate.functions.say == self.mpstate.functions.say:
            self.mpstate.functions.say = self.old_mpstate_say_function
        self.kill_speech_dispatcher()

    #
    # Different speech backends.
    #

    def say_using_speechd(self, text, priority='important',voice=None):
        '''speak some text'''
        ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
        import speechd
        self.speech = speechd.SSIPClient('MAVProxy%u' % os.getpid())
        self.speech.set_output_module('festival')
        self.speech.set_language('en')
        self.speech.set_priority(priority)
        self.speech.set_punctuation(speechd.PunctuationMode.SOME)
        self.speech.speak(text)
        self.speech.close()

    def say_using_espeak(self, text, priority='important',voice=None):
        '''speak some text using espeak'''
        from espeak import espeak
        if self.settings.speech_voice:
            espeak.set_voice(self.settings.speech_voice)
        espeak.synth(text)

    def say_using_speech(self, text, priority='important',voice=None):
        '''speak some text using speech module'''
        import speech
        speech.say(text)

    def say_using_say(self, text, priority='important', voice=None):
        '''speak some text using macOS say command'''
        if self.say_speech_backend is None:
            subprocess.call(['say', ""]) #Will throw an exception if say doesn't exist
            self.say_speech_backend = SaySpeechBackend()
            self.say_speech_backend.start()

        self.say_speech_backend.say(text, priority, voice)
        
    #
    # External Interface
    #

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if self.enabled_for == "NONE":
            return;
        type = msg.get_type()
        if type == "STATUSTEXT":
            # say some statustext values
            if msg.text.startswith("Tuning: "):
                self.say(msg.text[8:])
        else:
            for announcements in self.msgs_to_announce:
                announcements.dispatch(msg, self.say)

    def print_and_say(self, text, priority='important', voice=None):
        self.console.writeln(text)
        if self.enabled_for == "ALL":
            self.say(text, priority, voice=voice)
        elif self.enabled_for == "MODE" and len(text) > 4 and text[0:4] == "Mode":
            self.say(text, priority='important', voice="Daniel")

    def say(self, text, priority='important', voice=None):
        '''speak some text'''
        ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
        if self.settings.speech and self.say_backend is not None:
            self.say_backend(text, priority=priority, voice=voice)

    #
    # Command line configuration commands.
    # NOTE: Speech can be enabled or disabled and changed from main mpstate settings.
    #
    def cmd_speech(self, args):
        '''speech commands'''
        if len(args) < 1 or args[0] == "list":
            print "usage: speech <say|list|enable|add|remove>"
            print "  Call any command with no arguments for details"
            print ""
            print "Speech module will announce:", self.enabled_for
            print "Speech module knows how to report: "
            for idx, announcement in enumerate(self.msgs_to_announce):
                print "#%d: %s" % (idx, announcement)
        elif args[0] == "say":
            cmd_usage  = "usage: speech say <text to say>\n"
            cmd_usage += "  Say arbitrary text. Useful for testing"
            if len(args) < 2:
                print(cmd_usage)
                return
            self.say(" ".join(args[1::]))
        elif args[0] == "enable":
            cmd_usage  = "usage: speech enable (ALL|MODE|MAV|NONE)\n"
            cmd_usage += "  ALL  - reports all console and mavlink messages\n"
            cmd_usage += "  MODE - reports only mode changes and mavlink messages\n"
            cmd_usage += "  MAV  - reports only mavlink messages\n"
            cmd_usage += "  NONE - disables speech"
            if len(args) < 2:
                print cmd_usage
            else:
                command = args[1].upper()
                if (command in ["ALL", "MODE", "MAV", "NONE"]):
                    self.enabled_for = command
                    if args[1] == "NONE":
                        self.settings.set('speech', 0)
                    else:
                        self.settings.set('speech', 1)
                else:
                    print cmd_usage
        elif args[0] == "add":
            usage =  "usage: speech\n"
            usage += "add <msg_type>.<msg_entry> periodic          (every=<seconds>) (as=<say_as>) (scale=<scale>) (format=<formatstr>) (voice=<voice>) \n"
            usage += "add <msg_type>.<msg_entry> conditional       (every=<seconds>) (as=<say_as>) (scale=<scale>) (format=<formatstr>) (voice=<voice>) (max=<val>) (min=<val>)\n"
            usage += "add <msg_type>.<msg_entry> onchange          (every=<seconds>) (as=<say_as>) (scale=<scale>) (format=<formatstr>) (voice=<voice>) (threshold=<threshold>)"
            usage += "add <msg_type>.<msg_entry> periodic_onchange (every=<seconds>) (as=<say_as>) (scale=<scale>) (format=<formatstr>) (voice=<voice>) (threshold=<threshold>) (ratelimit=<limit in seconds>)"
            
            if len(args) < 3:
                print usage
            else:
                try:
                    msg_type, entry_name = args[1].split(".")
                    what_type = args[2].lower()

                    announce = None
                    if what_type == "periodic":
                        announce = PeriodicSpeechAnnounce(self, msg_type, entry_name)
                    elif what_type == "conditional":
                        announce = ConditionalSpeechAnnounce(self, msg_type, entry_name)
                    elif what_type == "onchange":
                        announce = OnChangeSpeechAnnounce(self, msg_type, entry_name)
                    elif what_type == "periodic_onchange":
                        announce = PeriodicOnChangeSpeechAnnounce(self, msg_type, entry_name)
                    else:
                        print "Need a type: periodic, conditional or onchange"
                        print usage
                        return

                    if len(args) > 2:
                        optargs = args[3::]
                        while len(optargs) > 0:
                            name, value = optargs[0].split("=")
                            if name == "every":
                                announce.set_every(float(value))
                            if name == "as":
                                announce.set_as(str(value))
                            if name == "scale":
                                announce.scale(float(value))
                            if name == "format":
                                announce.set_format(str(value))
                            if name == "max":
                                announce.set_max(float(value))
                            if name == "min":
                                announce.set_min(float(value))
                            if name == "threshold":
                                announce.set_threshold(float(value))
                            if name == "voice":
                                announce.set_voice(str(value))
                            if name == "ratelimit":
                                announce.set_ratelimit(float(value))
                            optargs = optargs[1::]

                    self.msgs_to_announce.append(announce)
                except Exception as e:
                    traceback.print_exc()
                    print usage

        elif args[0] == "remove":
            try:
                del self.msgs_to_announce[int(args[1])]
            except Exception:
                print "usage: speech remove <index>"
        else:
            print(usage)
            return


def init(mpstate):
    '''initialise module'''
    return SpeechModule(mpstate)
