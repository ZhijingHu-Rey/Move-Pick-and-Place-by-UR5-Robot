function[a]=pick(start,target,method,ur5)
    % convert frame to the gripper
    gt6=[0 -1 0 0;
        1 0 0 0;
        0 0 1 0.13;
        0 0 0 1];
    K=0.3;
    p=zeros([4,4]);
    p(3,4)=0.3;
    startup=start+p;
    targetup=target+p;
    home = startup;
    home(1:3, 4) = (startup(1:3, 4) + targetup(1:3, 4)+[0;0.3;0]) / 2;
                    
    % Choose RR control
    if(method=="RR")
        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','home',home);
        homeerr=ur5RRcontrol(home,0.1,ur5);
        if(homeerr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','startup',startup);
        startuperr=ur5RRcontrol(start+p,0.1,ur5);
        if(startuperr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','start',start);
        starterr=ur5RRcontrol(start,0.1,ur5);
        if(starterr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','startup',startup);
        startuperr=ur5RRcontrol(start+p,0.1,ur5);
        if(startuperr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','targetup',targetup);
        targetuperr=ur5RRcontrol(targetup,0.1,ur5);
        if(targetuperr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','target',target);
        targeterr=ur5RRcontrol(target,0.1,ur5);
        if(targeterr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','targetup',targetup);
        targetuperr=ur5RRcontrol(targetup,0.1,ur5);
        if(targetuperr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        homeerr=ur5RRcontrol(home,0.1,ur5);
        
    % Choose IK control
    elseif(method=="IK")
        qstart=ur5InvKin(start/gt6);
        qstartup=ur5InvKin(startup/gt6);
        qhome=ur5InvKin(home/gt6);
        qtarget=ur5InvKin(target/gt6);
        qtargetup=ur5InvKin(targetup/gt6);
        
        [bestqhome,homefound]=find_bestQ(qhome);
        [bestqstartup,startupfound]=find_bestQ(qstartup);
        [bestqstart,startfound]=find_bestQ(qstart);
        [bestqtargetup,targetupfound]=find_bestQ(qtargetup);
        [bestqtarget,targetfound]=find_bestQ(qtarget);
        if(homefound==-1||startupfound==-1||startfound==-1||targetupfound==-1||targetfound==-1)
            disp("IK cannot find a path");
            return
        end
        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','home',home);
        ur5.move_joints(bestqhome,20);

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','startup',startup);
        ur5.move_joints(bestqstartup,10);

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','start',start);
        ur5.move_joints(bestqstart,10);

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','startup',startup);
        ur5.move_joints(bestqstartup,10);
        q_st = ur5.get_current_joints() - ur5.home;
        gst_true = ur5FwdKin(q_st)*gt6;
        disp('start');
        disp(gst_true);

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','targetup',targetup);
        ur5.move_joints(bestqtargetup,10);

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','target',target);
        ur5.move_joints(bestqtarget,10);

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','targetup',targetup);
        ur5.move_joints(bestqtargetup,10);
        q_tr = ur5.get_current_joints() - ur5.home;
        gtr_true = ur5FwdKin(q_tr)*gt6;
        disp('target');
        disp(gtr_true);

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','home',home);
        ur5.move_joints(bestqhome,10);

    % Choose TJ control
    elseif(method=="TJ")
        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','home',home);
        homeerr=ur5RRTJcontrol(home,K,ur5);
        if(homeerr==-1) return; end
        
        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','startup',startup);
        startuperr=ur5RRTJcontrol(start+p,K,ur5);
        if(startuperr==-1) return; end
        
        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','start',start);
        starterr=ur5RRTJcontrol(start,K,ur5);
        if(starterr==-1) return; end
        
        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','startup',startup);
        startuperr=ur5RRTJcontrol(start+p,K,ur5);
        if(startuperr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','targetup',targetup);
        pickuperr=ur5RRTJcontrol(target+p,K,ur5);
        if(pickuperr==-1) return; end
        
        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','target',target);
        pickerr=ur5RRTJcontrol(target,K,ur5);
        if(pickerr==-1) return; end
        
        disp('Click in the figure');
        waitforbuttonpress;
        tf_frame('base_link','targetup',targetup);
        pickuperr=ur5RRTJcontrol(target+p,K,ur5);
        if(pickuperr==-1) return; end

        disp('Click in the figure');
        waitforbuttonpress;
        homeerr=ur5RRTJcontrol(home,K,ur5);
        if(homeerr==-1) return; end
    end
    disp('end');
end