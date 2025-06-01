
use heapless::binary_heap::{BinaryHeap, Max};
use heapless::spsc::Queue;
use bxcan::{Frame};
use core::cmp::Ordering;

#[derive(Debug)]
pub struct PriorityFrame(pub Frame);

/// Ordering is based on the Identifier and frame type (data vs. remote) and can be used to sort
/// frames by priority.
impl Ord for PriorityFrame {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.priority().cmp(&other.0.priority())
    }
}

impl PartialOrd for PriorityFrame {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for PriorityFrame {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for PriorityFrame {}



type RxBuf = Queue<Frame, 32>;
type TxBuf = BinaryHeap<PriorityFrame, Max, 16>;

pub struct CanRx {
    buf: RxBuf,
    npackets: usize,
    noverrun: usize,
}
impl CanRx {
    pub fn new() -> CanRx {
        CanRx{buf: RxBuf::new(), npackets: 0, noverrun: 0}
    }
    pub fn push(&mut self, frame: Frame) {
        let ok = match self.buf.enqueue(frame) {
            Ok(_r) => true,
            Err(_e) => false
        };
        if ok {
            self.npackets += 1;
        } else {
            self.noverrun += 1;
        }
    }
    pub fn pop(&mut self) ->Option<Frame> { 
        self.buf.dequeue()
    }
    pub fn is_empty(&mut self) ->bool {
        self.buf.is_empty()
    }
}


pub struct CanTx {
    buf: TxBuf,
    npackets: usize,
    noverrun: usize,

}

impl CanTx {
    pub fn new() -> CanTx {
        CanTx{buf: TxBuf::new(), npackets: 0, noverrun: 0}
    }
    pub fn push(&mut self, frame: Frame) {
        let ok = match self.buf.push(PriorityFrame(frame)) {
            Ok(_r) => true,
            Err(_e) => false
        };
        if ok {
            self.npackets += 1;
        } else {
            self.noverrun += 1;
        }
    }
    pub fn peek(&mut self) -> Option<&PriorityFrame> { 
        self.buf.peek()
    }
    pub fn pop(&mut self) { 
        self.buf.pop().unwrap();
    }

}

pub fn get_stm32_uid_as_u21() -> u32 {
    fn id_as_a32() -> [u32; 3] {
        let idbuf = stm32_device_signature::device_id();
        let mut ret  = [0u32, 0u32, 0u32];
        for i in 0..3 {
            let mut val = 0u32;
            for j in 0..4 {
                val += (idbuf[4*i + j] as u32) << (8*j);
            }
            ret[i] = val;
        }
        return ret;
    }


    let mut ret = 0u32;
    let inbuf = id_as_a32();

    let mut bit_offset = 0isize;

    for word in inbuf {

        let mut bits_remaining = 32isize;

        while bits_remaining > 0 {
            
            ret ^= (word << bit_offset) & 0x1F_FF_FF;
            
            let bits_used =
                if bits_remaining > 21isize - bit_offset { 
                    21isize - bit_offset
                } else {
                    bits_remaining
                };
            //let br0 = bits_remaining;
            //let bo0 = bit_offset;

            bits_remaining -= bits_used;
            bit_offset += bits_used;
            /*rtt_target::rprintln!("COMP w={:x} bu={} bo={}->{} br={}->{} {:x}",
                word,
                bits_used,
                bo0,
                bit_offset,
                br0,
                bits_remaining,
                ret);*/
            if bit_offset >= 21 {bit_offset = 0;}
        }

    }



    return ret;
}


