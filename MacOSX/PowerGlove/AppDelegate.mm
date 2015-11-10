//
//  AppDelegate.m
//  PowerGlove
//
//  Created by Robby on 2/16/15.
//  Copyright (c) 2015 Robby. All rights reserved.
//

#import "AppDelegate.h"
#import "BLEManager.h"

#include "OscOutboundPacketStream.h"
#include "UdpSocket.h"


#define ADDRESS "127.0.0.1"
#define PORT 7000

#define OUTPUT_BUFFER_SIZE 1024


@interface AppDelegate () <BLEDelegate> {
    BLEManager *bleManager;
    NSUInteger scanClock;
    NSTimer *scanClockLoop;
}

@property (weak) IBOutlet NSWindow *window;
@end

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
    // Insert code here to initialize your application
    
    bleManager = [[BLEManager alloc] initWithDelegate:self];
    
    
//    char buffer[OUTPUT_BUFFER_SIZE];
//    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
//    UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
//  
//    p << osc::BeginBundleImmediate
//    << osc::BeginMessage( "/test1" )
//    << true << 23 << (float)3.1415 << "let's" << osc::EndMessage
//    << osc::BeginMessage( "/test2" )
//    << true << 24 << (float)10.8 << "go!" << osc::EndMessage
//    << osc::EndBundle;
//    
//    transmitSocket.Send( p.Data(), p.Size() );

}

- (void)applicationWillTerminate:(NSNotification *)aNotification {
    // Insert code here to tear down your application
}

-(void)bootScanIfPossible{
    if([bleManager connectionState] == BLEConnectionStateDisconnected)
        [self startScan];
}
-(void) scanClockLoopFunction{
    if(scanClock >= 60){
        [bleManager stopScan];
        return;
    }
    scanClock++;
}
-(void) startScan{
    NSLog(@"startScan");
    scanClock = 0;
    scanClockLoop = [NSTimer scheduledTimerWithTimeInterval:1.0 target:self selector:@selector(scanClockLoopFunction) userInfo:nil repeats:YES];
    [[NSRunLoop currentRunLoop] addTimer:scanClockLoop forMode:NSRunLoopCommonModes];
    [bleManager startScanAndAutoConnect];
}

-(void) observeValueForKeyPath:(NSString *)keyPath ofObject:(id)object change:(NSDictionary *)change context:(void *)context{
    if([keyPath isEqualToString:@"orientation"]){
        //
    }
    else {
        [super observeValueForKeyPath:keyPath ofObject:object change:change context:context];
    }
}

#pragma mark- BLE DELEGATES

-(void) BLEBootedAndReady{
    [self bootScanIfPossible];
}
-(void) bluetoothStateDidUpdate:(BOOL)enabled{
    if(!enabled)
        NSLog(@"Bluetooth if off");
}
-(void) hardwareDidUpdate:(BLEHardwareState)state{
    BOOL capable = false;
    if(state == BLEHardwareStatePoweredOn){
        capable = true;
    }
    else if(state == BLEHardwareStateUnsupported){
        NSLog(@"Your computer doesn't have Bluetooth Low Energy");
    }
    else if(state == BLEHardwareStateUnauthorized){
        NSLog(@"The app is asking for permission to use Bluetooth Low Energy");
    }
    else if (state == BLEHardwareStatePoweredOff) {
        NSLog(@"Turn on Bluetooth Low Energy and try again");
    }
    else {
        NSLog(@"Bluetooth status unknown");
    }
}
-(void) connectionDidUpdate:(BLEConnectionState)state{
    if(state == BLEConnectionStateDisconnected){
        // if we just disconnected from a device
        if(scanClockLoop == nil){
            NSLog(@"disconnected from BLE device");// %@",[_peripheral name]]];
        }
        // if scanning ended unsuccessfully
        else if(scanClockLoop){
            [scanClockLoop invalidate];
            scanClockLoop = nil;
            NSLog(@"nothing in range");
        }
        else if(![bleManager isBluetoothEnabled]){
            NSLog(@"bluetooth is off");
        }
        else {
            NSLog(@"nothing in range");
        }
    }
    else if(state == BLEConnectionStateScanning){
        NSLog(@"searching for a connection..");
    }
    else if(state == BLEConnectionStateConnected){
        if(scanClockLoop){
            [scanClockLoop invalidate];
            scanClockLoop = nil;
        }
        NSLog(@"connected to BLE device");//%@",[_peripheral name]]];
    }
}
-(void) characteristicDidUpdate:(NSData *)data{
//    NSLog(@"%@",data);
    
    if(data.length == 9){
        unsigned char *msg = (unsigned char*)[data bytes];
        
        char buffer[OUTPUT_BUFFER_SIZE];
        osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
        UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
        
        //    float one = msg[0] + msg[1];
        p << osc::BeginBundleImmediate << osc::BeginMessage( "/quaternion/x" ) << msg[0] << osc::EndMessage
        << osc::BeginMessage( "/quaternion/y" ) << msg[1] << osc::EndMessage
        << osc::BeginMessage( "/quaternion/z" ) << msg[2] << osc::EndMessage
        << osc::BeginMessage( "/quaternion/w" ) << msg[3] << osc::EndMessage
        << osc::BeginMessage( "/accelerometer/x" ) << msg[4] << osc::EndMessage
        << osc::BeginMessage( "/accelerometer/y" ) << msg[5] << osc::EndMessage
        << osc::BeginMessage( "/accelerometer/z" ) << msg[6] << osc::EndMessage
        << osc::BeginMessage( "/flex/1" ) << msg[7] << osc::EndMessage
        << osc::BeginMessage( "/flex/2" ) << msg[8] << osc::EndMessage
        << osc::EndBundle;
        transmitSocket.Send( p.Data(), p.Size() );
    }

    if(data.length == 4){
    unsigned char *msg = (unsigned char*)[data bytes];
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
    
//    float one = msg[0] + msg[1];
    p << osc::BeginBundleImmediate << osc::BeginMessage( "/quaternion/x" ) << msg[0] << osc::EndMessage
    << osc::BeginMessage( "/quaternion/y" ) << msg[1] << osc::EndMessage
    << osc::BeginMessage( "/quaternion/z" ) << msg[2] << osc::EndMessage
    << osc::BeginMessage( "/quaternion/w" ) << msg[3] << osc::EndMessage
//    << (float)one << osc::EndMessage
//    << osc::BeginMessage( "/test2" )
//    << true << 24 << (float)10.8 << "way" << osc::EndMessage
    << osc::EndBundle;
    
    transmitSocket.Send( p.Data(), p.Size() );
        
    }
    if(data.length == 3){
        unsigned char *msg = (unsigned char*)[data bytes];
        char buffer[OUTPUT_BUFFER_SIZE];
        osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
        UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
        //    float one = msg[0] + msg[1];
        p << osc::BeginBundleImmediate << osc::BeginMessage( "/accelerometer/x" ) << msg[0] << osc::EndMessage
        << osc::BeginMessage( "/accelerometer/y" ) << msg[1] << osc::EndMessage
        << osc::BeginMessage( "/accelerometer/z" ) << msg[2] << osc::EndMessage
        << osc::EndBundle;
        transmitSocket.Send( p.Data(), p.Size() );
    }
    if(data.length == 2){
        unsigned char *msg = (unsigned char*)[data bytes];
        char buffer[OUTPUT_BUFFER_SIZE];
        osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
        UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
        //    float one = msg[0] + msg[1];
        p << osc::BeginBundleImmediate << osc::BeginMessage( "/flex/1" ) << msg[0] << osc::EndMessage
        << osc::BeginMessage( "/flex/2" ) << msg[1] << osc::EndMessage
        << osc::EndBundle;
        transmitSocket.Send( p.Data(), p.Size() );
    }

//    if ([data length] == 4) {  //([characteristic.value bytes]){
//        [analogStick updateOrientation:data];
//        if(_orientationWindowVisible){
//            [orientationView setOrientationMatrix:[analogStick m]];
//            [orientationView setNeedsDisplay:true];
//        }
//    }
//    else if ([data length] == 1){
//        unsigned char *msg = (unsigned char*)[data bytes];
//        if (*msg == 0x3b){ // exit code
//            NSLog(@"received exit code");
//            [bleManager disconnect];
//        }
//    }
//    else if ([data length] == 2) {
//        unsigned char *msg = (unsigned char*)[data bytes];
//        if(msg[0] == 0x88){ // button update code
//            // the only button we have so far: screen touch
//            BOOL t = msg[1];
//            NSLog(@"RECEIVED BUTTON UPDATE: %d",t);
//            [orientationView setScreenTouched:t];
//        }
//    }
}


@end
