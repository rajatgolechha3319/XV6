#include "types.h"
#include "param.h"
#include "defs.h"
#include "mmu.h"
#include "memlayout.h"
#include "x86.h"
#include "proc.h"
#include "fs.h"

#define SWAPSIZE SWAPBLOCKS
#define IRON_DOME PTXSHIFT
#define ONE 1
#define ZERO 0

struct s1{
    int attribute_1;
    int attribute_2;
    pte_t* pte_array[64];
    uint refC;
};

int reducer(int num){
    return (num - ONE - ONE) / 8;
}

uint rmap[PHYSTOP >> IRON_DOME];
pte_t* reverse_map[PHYSTOP >> IRON_DOME][64];


void inc_rmap(pte_t* pte){
    uint pa = PTE_ADDR(*pte);
    int refC = rmap[pa >> IRON_DOME];
    reverse_map[pa >> IRON_DOME][refC] = pte;
    // reverse_map_pid[pa >> IRON_DOME][refC] = pid;
    rmap[pa >> IRON_DOME] = rmap[pa >> IRON_DOME] + ONE;
}

void dec_rmap(pte_t* pte){
    uint phy = PTE_ADDR(*pte);
    int i = ZERO;
    int rC = rmap[phy >> IRON_DOME];
    while(i< rC){
        if(reverse_map[phy >> IRON_DOME][i] == pte){
            break;
        }
        i = i + ONE;
    }
    if(i == rC){
        panic("Reverse map not found in dec rmap\n");
    }
    rmap[phy >> IRON_DOME] = rmap[phy >> IRON_DOME] - ONE;
    while(i < rC-ONE){
        reverse_map[phy >> IRON_DOME][i] = reverse_map[phy >> IRON_DOME][i+ONE];
        i = i + ONE;
    }
}

uint get_rmap(uint pa){
    return rmap[pa >> IRON_DOME];
}

void set_rmap(uint pa){
    rmap[pa >> IRON_DOME] = ZERO;
}

struct s1 swap_table[SWAPSIZE/(PGSIZE/BSIZE)];


void inc_swap_table(pte_t* pte1 , pte_t* pte , int rand){
    // push this pte in the swap table
    if(rand == ONE){
        panic(" NEVER TO REACH HERE ONLY FOR DEBUGGING \n");
    }
    uint block_no = reducer(*pte1 >> IRON_DOME);
    swap_table[block_no].pte_array[swap_table[block_no].refC] = pte;
    swap_table[block_no].refC = swap_table[block_no].refC + ONE;
}


void pageswapinit(void){
    int i = ZERO;
    int limit = SWAPSIZE/(PGSIZE/BSIZE);
    while(i < limit){
        swap_table[i].attribute_1 = ONE;
        swap_table[i].attribute_2 = 2 + i*(PGSIZE/BSIZE);
        swap_table[i].refC = ZERO;
        // INITIALIZING THE PTE ARRAY
        int j = ZERO;
        while(j < 64){
            swap_table[i].pte_array[j] = ZERO;
            j = j + ONE;
        }
        i = i + ONE;
    }
    // cprintf("Swap table initialized\n");
}

void swapout_helper(uint pa, int block){
    int rC = rmap[pa >> IRON_DOME];
    swap_table[block].refC = rC;
    int block_num = swap_table[block].attribute_2;
    int i = ZERO;
    while( i < rC){
        swap_table[block].pte_array[i] = reverse_map[pa >> IRON_DOME][i];
        uint flags = PTE_FLAGS(*reverse_map[pa >> IRON_DOME][i]);
        *reverse_map[pa >> IRON_DOME][i] = (block_num << IRON_DOME);
        *reverse_map[pa >> IRON_DOME][i] |= flags;
        *reverse_map[pa >> IRON_DOME][i] |= PTE_SWAPPED;
        *reverse_map[pa >> IRON_DOME][i] &= (~PTE_P);
        i = i + ONE;
    }
    rmap[pa >> IRON_DOME] = ZERO;
}

void swapin_helper(uint pa, int block){
    int rC = swap_table[block].refC;
    rmap[pa >> IRON_DOME] = rC;
    int i = ZERO;
    while(i < rC){
        reverse_map[pa >> IRON_DOME][i] = swap_table[block].pte_array[i];
        uint flags = PTE_FLAGS(*swap_table[block].pte_array[i]);
        *swap_table[block].pte_array[i] = pa;
        *swap_table[block].pte_array[i] |= flags;
        *swap_table[block].pte_array[i] |= PTE_P;
        *swap_table[block].pte_array[i] &= (~PTE_SWAPPED);
        i = i + ONE;
    }
}

static pte_t*
walkpgdir(pde_t *pgdir, const void *va, int alloc)
{
  pde_t *pde;
  pte_t *pgtab;

  pde = &pgdir[PDX(va)];
  if(*pde & PTE_P){
    pgtab = (pte_t*)P2V(PTE_ADDR(*pde));
  } else {
    if(!alloc || (pgtab = (pte_t*)kalloc()) == ZERO)
      return ZERO;
    // Make sure all those PTE_P bits are zero.
    memset(pgtab, ZERO, PGSIZE);
    // The permissions here are overly generous, but they can
    // be further restricted by the permissions in the page table
    // entries, if necessary.
    *pde = V2P(pgtab) | PTE_P | PTE_W | PTE_U;
  }
  return &pgtab[PTX(va)];
}

void swap_page_out(){
    pte_t* SQUIRTLE = page_replacement();
    // Need to decrease RSS for all the processes using this page
    rss_decrementer(PTE_ADDR(*SQUIRTLE));
    // cprintf("Swap out page %x\n", PTE_ADDR(*pte));
    // Now swap out the page pointed by pte
    int i = ZERO;
    int limit = SWAPSIZE/(PGSIZE/BSIZE);
    while( i < limit){
        if(swap_table[i].attribute_1 == ONE){
            break;
        }
        i = i + ONE;
    }
    if(i == (SWAPSIZE/(PGSIZE/BSIZE))){
        panic("Swap space full\n");
    }
    swap_table[i].attribute_1 = ZERO;
    // swap_table[i].page_perm = PTE_FLAGS(*pte);
    uint phys_addr = PTE_ADDR(*SQUIRTLE);
    page_disk_interface((char*)P2V(PTE_ADDR(*SQUIRTLE)), swap_table[i].attribute_2,ZERO);
    swapout_helper(phys_addr, i);
    kfree((char*)P2V(phys_addr));
    // cprintf("Page %x swapped out to block %d\n", (pte), swap_table[i].attribute_2);
}

void flush(pte_t* page){
    int block_num = *page >> IRON_DOME;
    int swap_block = reducer(block_num);
    // Iterate and find the page in the swap table
    int i = ZERO;
    int limit = swap_table[swap_block].refC;
    while(i < limit){
        if(swap_table[swap_block].pte_array[i] == page){
            swap_table[swap_block].pte_array[i] = ZERO;
            break;
        }
        i = i + ONE;
    }
    if(i == swap_table[swap_block].refC){
        panic("Page not found in swap table\n");
    }
    // shift the remaining pages back
    swap_table[swap_block].refC = swap_table[swap_block].refC - ONE;
    limit = swap_table[swap_block].refC;
    while( i < limit){
        swap_table[swap_block].pte_array[i] = swap_table[swap_block].pte_array[i+ONE];
        i = i + ONE;
    }
    if(swap_table[swap_block].refC == ZERO){
        swap_table[swap_block].attribute_1 = ONE;
    }
    
}
void case_swap(uint va, struct proc* p, pte_t* pte){
    // cprintf(" SWAP IN \n");
    // uint va = rcr2();
    // pde_t *pgdir = myproc_pg()->pgdir;
    // pte_t *pte = walkpgdir(pgdir, (void*)va, 0);
    int block_num = *pte >> IRON_DOME;
    // cprintf("pagefault_handler: loading page from disk\n");
    // myproc_pg()->rss++;
    // char *page = kalloc();
    // if(page == 0){
    //     swap_page_out();
    // }
    // page = kalloc();
    char* flareon = kalloc();
    page_disk_interface(flareon, block_num,ONE);



    // cprintf("pagefault_handler: page loaded\n");
    // // update the page table entry
    int swap_block = reducer(block_num);
    swapin_helper(V2P(flareon), swap_block);

    // increment the rss for all the processes using this page
    rss_incrementer(V2P(flareon));


    // // free the swap slot
    swap_table[swap_block].attribute_1 = ONE;
    return;
}

void case_cow(uint va, struct proc* p, pte_t* pte){
        if(*pte & PTE_P){
            if(*pte & PTE_W){
                panic(" Page fault on a writable page\n");
            }
            uint pa = PTE_ADDR(*pte);
            int refC = get_rmap(pa);
            uint flags = PTE_FLAGS(*pte);
            if(refC == ONE){
                *pte |= PTE_W;
                return;
            }
            else{
                dec_rmap(pte);
                // Allocate a new page
                char* new_page = kalloc();
                // Copy the contents of the old page to the new page
                memmove(new_page, (char*)P2V(pa), PGSIZE);
                // Update the page table entry
                *pte = V2P(new_page) | flags;
                *pte |= PTE_W;
                inc_rmap(pte);
                lcr3(V2P(p->pgdir));
                return;
            }
        }
}

void page_fault(){
    
    uint va = rcr2();
    struct proc* p = myproc();
    pte_t* pte = walkpgdir(p->pgdir, (void*)va, ZERO);
    if(*pte & PTE_SWAPPED){
        case_swap(va, p, pte);
    }
    else{
        case_cow(va, p, pte);
    }
}