#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, UC Regents
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of California nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib
#roslib.load_manifest('starmac_tools')
import rosbag
import rospy
import sys
import numpy as np
import scipy.io as sio
import gzip
import cPickle
import time

_primitive_types = [ 'byte', 'int8', 'uint8',
                    'int16', 'uint16', 'int32', 'uint32',
                    'int64', 'uint64', 'float32', 'float64', 'bool']

def mangle(s):
    """
    Function to mangle field names, replacing first underscores with double underscores,
    then slashes with underscores
    """
    mangled = s.replace('_','__').replace('/','_')
    assert(unmangle(mangled)) == s
    return mangled

def unmangle(s):
    return s[::-1].replace('__','~')[::-1].replace('_', '/').replace('~','_')

class BagLoader(object):
    PICKLED_EXT = '.sbag.gz'
    """
    Class that allows one to load data from a ROS .bag file. 
    
    For the impatient:
    >>> mybag = BagLoader('some_bagfile.bag')
    >>> t = mybag._grey_innerloop_time
    >>> V = mybag._grey_innerloop_battVoltage
    >>> from pylab import *
    >>> plot(t, V)    
    
    Data is stored in the instance private member
    dictionary _data, with keys that are based on the ROS 'name' of each time-series variable (the ROS topic name
    as recorded in the bagfile is concatenated with slashes with the field name, i.e. /namespace/topic/field). 
    The variables are also available as instance members, with name mangling to convert slashes to underscores.
    (this is handy when you have some tab completion as in IPython)
    
    Note that at present, 'nested' fields, i.e. toplevel fields that are not a ROS 'primitive type' are not supported
    and are ignored, with the exception of fields of type Header in the top level, for which the stamp field is retrieved
    as a special case.
    
    Each topic includes the special field 'time' -- note that this is the time at which the message was recorded
    by rosbag. If the message includes a Header field, then at present the timestamp within cannot be accessed,
    as nested variables are not yet handled.
    
    Another useful feature is the ability to save the data to a Matlab-readable .mat file:
    >>> mybag.save_mat('test.mat')
    
    Note - the minimum recorded timestamp in the bagfile is taken to be time zero, and the message and header (when present)
    timestamps subtract this value (self.start_stamp) in 64-bit integer arithmetic before storing the result as a double. Thus
    the time values in the output are double (64-bit float) times in nanoseconds with zero being the aforementioned quantity.
    Note that this could lead to large negative times in cases where the timestamp is set to zero (may happen in headers). It is
    the user's responsibility to handle such a case appropriately. 
    """
    def __init__(self, filename=None, verbose=False, skip_images=True):
        self.verbose = verbose
        self._data = {} # this is where everything will go
        self._dict_names = {} # will contain the 'mangled' names as keys 
        self._ros_types = {}
        if filename is not None:
            if filename.lower().endswith('.mat'):
                self.load_mat(filename)
            else:
                self._load(filename)
        self.filename = filename
        
    def _save_pickled(self, filename):
        """
        Save a pickled and gzipped representation of the loaded data. Unfortunately this seems to run very slowly..
        """
        if not filename.endswith(BagLoader.PICKLED_EXT):
            print('Warning: Recommended filename extension is %s' % BagLoader.PICKLED_EXT)
        stuff = {'___VERSION_MAJOR__': 0, # increment this whenever format changes and breaks backwards compatibility
                 '___VERSION_MINOR__': 1, # increment this whenever format changes
                 '_data': self._data,
                 '_dict_names': self._dict_names,
                 '_ros_types': self._ros_types,
                 'topic_msg_counts': self.topic_msg_counts,
                 'start_stamp': self.start_stamp,
                 }
        fh = gzip.open(filename, 'w')
        cPickle.dump(stuff, fh)
        fh.close()
        
    def _load_pickled(self, filename):
        """
        Load a pickled and gzipped representation of the data. Unfortunately this seems to run very slowly..
        """
        if not filename.endswith(BagLoader.PICKLED_EXT):
            print('Warning: Recommended filename extension is %s' % BagLoader.PICKLED_EXT)
        fh = gzip.open(filename, 'r')
        stuff = cPickle.load(fh)
        fh.close()
        assert(stuff['___VERSION_MAJOR__'] == 0)
        self._dict_names = stuff['_dict_names']
        self._ros_types = stuff['_ros_types']
        self.topic_msg_counts = stuff['topic_msg_counts']
        self.start_stamp = stuff['start_stamp']
        self._data = stuff['_data']
        for dict_name, name in self._dict_names.items():
            self.__dict__[dict_name] = self._data[name]

    def _load(self, filename):
        # Load it up:
        print "Loading data from", filename
        self.b = rosbag.Bag(filename)
        # Figure out what time zero should be:
        self.start_stamp  = min([index[ 0].time for index in self.b._connection_indexes.values()])
        # Get the list of topics:
        topics = set([c.topic for c in self.b._get_connections()])                
        topic_datatypes    = {}
        topic_conn_counts  = {}
        topic_msg_counts   = {}
        topic_freqs_median = {}
        # And figure out a bunch of stuff about them:
        self.topic_msg_counts = {}
        for topic in topics:
            connections = list(self.b._get_connections(topic))
            stamps = [entry.time.to_sec() for entry in self.b._get_entries(connections)]
            
            topic_datatypes[topic] = connections[0].datatype
            topic_conn_counts[topic] = len(connections)
            self.topic_msg_counts[topic] = len(stamps)
        #    if len(stamps) > 1:
        #        periods = [s1 - s0 for s1, s0 in zip(stamps[1:], stamps[:-1])]
        #        med_period = _median(periods)
        #        if med_period > 0.0:
        #            topic_freqs_median[topic] = 1.0 / med_period
        
        
        for topic in topics:
            if self.verbose: print "Loading topic %s, type %s" % (topic, topic_datatypes[topic]),
            if topic_datatypes[topic] == 'sensor_msgs/Image':
                if self.verbose: print ".. Skipping"
            else:
                if self.verbose: print 
                self._load_process_topic(topic)
                
    def _new_data(self, name, init_data, ros_type=None):
        """
        Initiaize a spot in self._data for the given name (topic+field), with 
        """
        self._data[name] = init_data
        self.__dict__[mangle(name)] = self._data[name]
        self._dict_names[mangle(name)] = name
        self._ros_types[name] = ros_type
                
    def _load_process_topic(self, topic):
        msg_iter = self.b.read_messages(topic)
        i = 0
        N = self.topic_msg_counts[topic]
        #(topic, msg, time) = msg_iter.next()
        for (topic, msg, time) in msg_iter:
            if i == 0:
                #print 'Topic: ', topic, '(', msg._type, ')'
                (fieldnames, types) = msg.__slots__, msg._get_types()
#                self._data[topic+'/_time'] = np.zeros(shape=(N,), dtype=np.float64)
#                self.__dict__[mangle(topic+'/_time')] = self._data[topic+'/_time']
                self._new_data(topic+'/_time', np.zeros(shape=(N,), dtype=np.float64))
                if ('header','Header') in zip(fieldnames,types):
#                    self._data[topic+'/_header_time'] =  np.zeros(shape=(N,), dtype=np.float64)
#                    self.__dict__[mangle(topic+'/_header_time')] = self._data[topic+'/_header_time']
                    self._new_data(topic+'/_header_time', np.zeros(shape=(N,), dtype=np.float64))
                    has_header = True
                else:
                    has_header = False
            header_time = self._sub_load_init_array(i, 0, topic, msg, time, N, get_header=has_header)
            self._data[topic+'/_time'][i] = (time-self.start_stamp).to_sec()
            if has_header:
                self._data[topic+'/_header_time'][i] = (header_time-self.start_stamp).to_sec()
            i += 1

    
    def _sub_load_init_array(self, i, depth, base, msg, time, N, get_header=False):
        verbose = self.verbose
        (fieldnames, types) = msg.__slots__, msg._get_types()
        if get_header:
            header_time = msg.__getattribute__('header').stamp
        else:
            header_time = None
        for fieldname2, type2 in zip(fieldnames, types):
            t_stripped = type2.split('[')[0]
            is_primitive = t_stripped in _primitive_types
            is_array = ('[' in type2 and ']' in type2)
            full_field_name = base + '/' + fieldname2
            array_size = None
            if type2 is 'time':
                for s in ('/secs', '/nsecs'):
                    if i == 0:
                        self._new_data(full_field_name + s, np.zeros(shape=(N,), dtype=np.uint32), (t_stripped, is_primitive, is_array, array_size))
                        if verbose: print full_field_name + s, ':', str(self._data[full_field_name+s].dtype), str(self._data[full_field_name+s].shape)
                self._data[full_field_name + '/secs'][i] = msg.__getattribute__(fieldname2).secs
                self._data[full_field_name + '/nsecs'][i] = msg.__getattribute__(fieldname2).nsecs
            else:
                if is_primitive:
                    if is_array:
                        array_size_str = type2.split('[')[1].split(']')[0]
                        if len(array_size_str) > 0: # fixed length array
                            array_size = int(array_size_str)
                            if i == 0:
                                self._new_data(full_field_name, np.zeros(shape=(N,array_size), dtype=t_stripped), 
                                               (t_stripped, is_primitive, is_array, array_size))
                                if verbose: print " "*depth + full_field_name, ':', str(self._data[full_field_name].dtype), str(self._data[full_field_name].shape)
                            self._data[full_field_name][i,:] = msg.__getattribute__(fieldname2)
                        else:
                            if i == 0:
                                array_size = None # variable length array
                                self._new_data(full_field_name, [], (t_stripped, is_primitive, is_array, array_size))
                                if verbose: print " "*depth + full_field_name, ':', type2
                            if t_stripped == 'uint8':
                                self._data[full_field_name].append(np.fromstring(msg.__getattribute__(fieldname2), dtype='uint8'))
                            else:
                                self._data[full_field_name].append(msg.__getattribute__(fieldname2))
                    else:
                        if i == 0:
                            self._new_data(full_field_name, np.zeros(shape=(N,), dtype=t_stripped), 
                                           (t_stripped, is_primitive, is_array, array_size))
                            if verbose: print " "*depth + full_field_name, ':', str(self._data[full_field_name].dtype), str(self._data[full_field_name].shape)
                        self._data[full_field_name][i] = msg.__getattribute__(fieldname2)
                elif t_stripped == 'string':
                    if i == 0:
                        self._new_data(full_field_name, [], (t_stripped, is_primitive, is_array, array_size))
                        if verbose:
                            if is_array:
                                print(" "*depth + full_field_name, ': string')
                            else:
                                print(" "*depth + full_field_name, ': string')
                    self._data[full_field_name].append(msg.__getattribute__(fieldname2))
                else:
                    if is_array:
                        if i == 0:
                            self._new_data(full_field_name, [], (t_stripped, is_primitive, is_array, array_size))
                            if verbose: print " "*depth + full_field_name, type2
                            print "WARNING: Arrays of non-primitive types not yet fully supported (%s : %s)" % (fieldname2, type2)
                        else:
                            self._data[full_field_name].append(msg.__getattribute__(fieldname2))
                    else:
                        if i == 0:
                            if verbose: print " "*depth + full_field_name, type2, "*"
                        self._sub_load_init_array(i, depth+1, full_field_name, msg.__getattribute__(fieldname2), time, N)
                            #print " "*depth + "<<< done recursing on ", full_field_name
        return header_time
                
    def save_mat(self, mat_filename):
        """
        Save data to a MATLAB (version 5) .mat file. Note that not all data can be saved in this format; basically only
        fields that are primitive types or arrays of primitive types (besides strings) are saved.
        """
        mdict = {}
        for k, v in self._data.iteritems():
            if v is not None:
                rtypes = self._ros_types[k]
                if rtypes is None:
                    is_primitive = True # special case to handle derived header time
                    is_string = False
                    is_array = False
                    array_size = 0
                else:
                    (t_stripped, is_primitive, is_array, array_size) = self._ros_types[k]
                    is_string = (t_stripped == 'string')
                if is_primitive:
                    mdict['v'+mangle(k)] = v # Matlab variable names must start with a letter
                elif is_string and not is_array:
                    mdict['v'+mangle(k)] = self._strlist_to_cellarray(v)
                else:
                    print("Warning: cannot save field %s to matlab (not a primitive type)" % k)
        stuff = {'info_VERSION_MAJOR__': 0, # increment this whenever format changes and breaks backwards compatibility
         'info_VERSION_MINOR__': 1, # increment this whenever format changes
         'info_topic_msg_counts': self._dict_to_cellarray(self.topic_msg_counts)
         }
        mdict.update(stuff)
        sio.savemat(mat_filename, mdict, long_field_names=True, oned_as='column')
        
    def load_mat(self, mat_filename):
        """
        Load data from a MATLAB (version 5) .mat file
        """
        mdict = sio.loadmat(mat_filename, squeeze_me=True, struct_as_record=False)
        assert(mdict['info_VERSION_MAJOR__'] == 0)
        for k, v in mdict.iteritems():
            #print(k)
            if k.startswith('v'):
                name = unmangle(k[1:])
                #print(name)
                if np.issubdtype(v[0].dtype, np.unicode): # detect string list
                    self._new_data(name, [str(vv.flat[0]) for vv in v])
                else:
                    self._new_data(name, v) # note that ros type is lost..
            elif k == 'info_topic_msg_counts':
                self.topic_msg_counts = self._cellarray_to_dict(mdict[k])
                
    def _dict_to_cellarray(self, d):
        """
        Helper function for save_mat
        """
        obj_arr = np.zeros((len(d),), dtype=np.object)
        i = 0
        for k, v in d.iteritems():
            obj_arr[i] = {'key': k, 'value': v}
            i += 1
        return obj_arr
    
    def _cellarray_to_dict(self, c):
        """
        Helper function for load_mat
        """
        d = {}
        for i in range(len(c)):
            cell = c[i]
            key = str(cell.key)
            value = cell.value.flat[0]
            #print key, value
            d[key] = value
        return d
    
    def _strlist_to_cellarray(self, sl):
        """
        Helper function for save_mat
        """
        ca = np.zeros((len(sl),), dtype=np.object)
        for i in range(len(sl)):
            ca[i] = sl[i]
        return ca
    
        
    def get_topics(self):
        temp = self.topic_msg_counts.keys()
        temp.sort()
        return temp
            
if __name__ == "__main__":
    start = time.time()
    bl = BagLoader(sys.argv[1])
    end = time.time()
    print("Loading from .bag took %f seconds" % (end - start))

    from pylab import *
    try:
        t = bl.estimator_output___time
        h = -bl.estimator_output_pose_pose_position_z
    except AttributeError: # kludge..
        t = bl.downlink_estimator_output___time
        h = -bl.downlink_estimator_output_pose_pose_position_z
    plot(t,h)
    title('Altitude vs. Time')
    xlabel('Time [s]')
    ylabel('Altitude [m]')

    print("Testing save to .mat ...")
    start = time.time()
    bl.save_mat('test.mat')
    end = time.time()
    print("Saving to .mat took %f seconds" % (end - start))

    print("Testing load from .mat ...")
    start = time.time()
    bl2 = BagLoader()
    bl2.load_mat('test.mat')
    end = time.time()
    print(bl.topic_msg_counts)
    print(bl2.topic_msg_counts)
    print("Loading from .mat took %f seconds" % (end - start))

    print("Testing save to .sbag.gz ...")
    start = time.time()
    bl._save_pickled('test.sbag.gz')
    end = time.time()
    print("Saving to .sbag.gz took %f seconds" % (end - start))
    
    print("Testing load from .sbag.gz ...")
    bl3 = BagLoader()
    start = time.time()
    bl3._load_pickled('test.sbag.gz')
    end = time.time()
    print("Loading from .sbag.gz took %f seconds" % (end - start))
    
    show()

