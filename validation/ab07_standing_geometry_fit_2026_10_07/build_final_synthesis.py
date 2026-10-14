"""Assemble the final local synthesis from frozen, already reviewed results."""
import datetime as dt
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
AUDIT = Path(__file__).resolve().parent
read = lambda name: json.loads((AUDIT/name).read_text())
j = read('analysis/lean_numeric_summary.json')
geometry = read('geometry/four_roi_support/fit_receipt.json')
peer = read('review/final_postprocessing_peer_review.json')
sea_peer = read('review/final_SEA_limit_peer_review.json')
claude_receipt = read('claude/SEA_supplement_completion_and_visible_read_receipt.json')
assert peer['status'] == 'PASS' and sea_peer['status'] == 'PASS'
assert any(x.get('subtype') == 'success' and not x.get('is_error')
           for x in claude_receipt['metadata'])
main = j['cases']['STANDING_FOURROI_AB07']
control = j['cases']['PYLON400_REFERENCE']
old = j['cases']['STANDING_AB07']
assert main['primary_full_three_cycle_statistics'] is None
assert old['primary_full_three_cycle_statistics'] is None
primary = control['primary_full_three_cycle_statistics']
common = j['secondary_common_observed_prefix']
parts = []
def para(value): parts.append(value.strip() + '\n')

para('AB07: calibrazione anatomica statica, preparazione e confronto con tibia_pylon400mm\n2026-10-07 — Sintesi locale dei risultati e del confronto con Claude Fable5.1, effortmax.')
para('Esito: la variante nuova rispetta ginocchio e appoggio geometrico del piede nella posa statica scelta. Il suo test dinamico si interrompe dopo0,16s per il superamento del limite superiore del ginocchio. Il riferimento400mm completa6,11s. Le prestazioni sui tre cicli e il detector sono misurabili solo nel riferimento; il confronto diretto tra varianti è limitato ai primi159ms osservati da entrambe. La calibrazione del detector è rimasta identica.')
para('Criterio di costruzione e dimensioni')
para('Il riferimento sano AB07 è la mediana delle23 coordinate delle436 righe del file statico originale, non una nuova posa ideale con tutte le articolazioni a zero. Le19 coordinate biologiche e il bacino sono preservati. La quota del ginocchio sano è474,955700mm. Il suolo della calibrazione è il pianoY=0 codificato nel modello: questo risultato è condizionato a tale piano e non dimostra una quota del pavimento misurata indipendentemente o un equilibrio statico sotto carico.')
para('Il piede comune AM_foot_l.STL è identico per tutti i casi. Essendo una superficie curva, «appoggio completo» è stato verificato con quattro regioni plantari: tallone e avampiede, ciascuno sul lato mediale e laterale. La variante finale ha tre vertici a terra non allineati, nessuna penetrazione della mesh e il quarto minimo regionale a0,000845mm, entro la tolleranza geometrica fissata di0,01mm. Non è una misura di pressione plantare.')
para('Per collegare i componenti mantenendo la quota del ginocchio: tibia_pylon433,966724mm; traslazioneY dell’attacco osseo_parent−185,770409mm; orientazioneX del medesimo attacco−3,318315°. Gli altri offset sono preservati. La rotazione riguarda il montaggio dell’intera protesi e non il detector o gli assi interni SEA. La traslazione osseo_parent non è la lunghezza del corpo osseo_pylon: il collegamento interno osseo_pylon→ginocchio di205mm rimane invariato. Sono state dimensionate le distanze cinematiche dei giunti; mesh strutturali, masse, centri di massa e inerzie non sono stati rifabbricati o riscalati.')
para('La mappatura statica della caviglia usa un offset−3,976520°, con caviglia protesica3,106664° per la posa sana7,083184°. Il ginocchio protesico mantiene la posa sana−7,045167° e la stessa altezza. La soluzione geometrica è stata decisa senza leggere reserve, eventi o output dinamici.')
para('Controllo400mm: nella stessa posa statica di riferimento il ginocchio è a472,552432mm e la mesh resta sopraY=0: minimo tallone16,018148mm, avampiede44,333093mm. La sua mappatura caviglia usa invece+4,730649°. La stabilità del banco con400mm non dimostra quindi che questa sia la lunghezza anatomica corretta secondo il criterio richiesto.')
para('È conservata anche una soluzione intermedia solo tallone/avampiede:432,914561mm, orientazione superiore0°, offset caviglia−3,606419°. Aveva appoggio mediale, ma gap laterali3,111808 e4,924960mm. Per soddisfare l’appoggio su entrambi i lati è stata aggiunta la variante finale a quattro regioni mediante un protocollo geometrico supplementare congelato prima della sua preparazione e simulazione.')

para('Pipeline e test effettivamente eseguiti')
para('Ogni variante usa la propria preparazione coerente: file statico originale con sole due coordinate protesiche mappate; calibrazione dei12 marker con gli helper originali; nuova IK completa su28612 pose da10,360 a153,415s con TRC e supporti GRF originali. La registrazione automatica al suolo derivata dal donatore è stata esclusa per preservare i vincoli del fit. Scaling, RRA e ID originali non sono stati rieseguiti. Per400mm modello, statico e IK della preparazione fresca riproducono byte per byte il riferimento storicoP3_A1.')
para('Il test è una simulazione forward CMC-like con riferimento protesico dalla IK propria del caso, azione prescribed_action e SEA fisico non-impedance. Non è un rollout della rete. Biologia muscle-driven con reserve residuali, forza di contatto virtuale applicata al lato sinistro e GRF misurata prescritta al destro; configurazione e contratto comuni conservati. Seed123, reset12,591740519774s, integrazione1ms, passo di controllo10ms, setup9s, episodio richiesto6,11s. Guardie immutate: ginocchio[-2,0]rad, caviglia[-0,9,+0,9]rad e penetrazione28mm. Il guard di tempo reale originale60s per segmento è rimasto attivo.')
para('La variante finale esegue16 passi di controllo e si arresta a0,16s con joint_divergence:pros_knee_angle, ginocchio finale+0,002520282rad=+0,144401°. Non è un timeout o un arresto per penetrazione: profondità massima cammino24,32755mm. La variante intermedia si arresta allo stesso limite articolare dopo17 passi,0,17s,+0,397352°. Non sono stati fatti tentativi dinamici con limiti, forze o detector ritoccati.')
para('Il riferimento400mm completa611 passi,6,11s. I17 fileSTO, il CSV eventi e tutte le611 righe della trace coincidono byte per byte conP3_A1, senza campi esclusi. Gli istanti finali di stato, runner e ambiente coincidono con il termine richiesto.')

para('Reserve: confronto diretto sull’avvio comune')
para(f'Finestra comune {common["range_s"][0]:.8f}–{common["range_s"][1]:.8f}s, durata{common["duration_s"]:.3f}s,160 campioni. La simulazione termina dopo0,16s ma registra lo stato prima di ogni sottopasso: il supporto effettivamente salvato della variante finale è0,159s. Il tempo di integrazione e il supporto dei dati non sono intercambiabili.')
rows = ['Caso\tReserve pelvis_ty media[N]\tRMS[N]\tImpulso assoluto[Ns]\tLavoro assoluto[J]\tForza sinistra media[N]']
for name, label in [('PYLON400_REFERENCE','400mm'),('STANDING_AB07','432,915mm /2regioni'),('STANDING_FOURROI_AB07','433,967mm /4regioni')]:
    c = common['cases'][name]
    rows.append(f'{label}\t{c["pelvis_vertical_reserve_force"]["mean"]:+.6f}\t{c["pelvis_vertical_reserve_force"]["RMS"]:.6f}\t{c["pelvis_vertical_reserve_impulse_Ns"]["absolute"]:.6f}\t{c["pelvis_vertical_reserve_work_J"]["absolute"]:.6f}\t{c["left_actual_force"]["mean"]:.6f}')
para('\n'.join(rows))
para('Il segno negativo della reserve pelvis_ty indica una forza verso il basso. La nuova variante produce un carico sinistro elevato già all’avvio, mentre400mm in questa stessa fase iniziale ha quasi assenza di contatto sinistro e reserve verso l’alto. RMS, impulso e lavoro mostrano maggiore assistenza richiesta dalla variante calibrata nell’avvio. Questo non è un confronto tra tre cicli stazionari e non va extrapolato a un’intera camminata.')

para('Reserve nel riferimento400mm sui tre cicli primari')
para(f'Finestra congelata {primary["observed_range_s"][0]:.12f}–{primary["observed_range_s"][1]:.12f}s, durata{primary["observed_duration_s"]:.12f}s. Per pelvis_ty: media{primary["pelvis_vertical_reserve"]["mean_N"]:+.6f}N; RMS{primary["pelvis_vertical_reserve"]["RMS_N"]:.6f}N; impulso assoluto{primary["pelvis_vertical_reserve"]["impulse_Ns"]["absolute"]:.6f}Ns; lavoro assoluto{primary["pelvis_vertical_reserve"]["work_J"]["absolute"]:.6f}J. Forza sinistra media{primary["left_actual_mean_N"]:.6f}N, richiesta dal bilancio del moto{primary["left_momentum_required_mean_N"]:.6f}N. RMS tracking ginocchio{primary["prosthetic_knee_RMS_deg"]:.6f}°, caviglia{primary["prosthetic_ankle_RMS_deg"]:.6f}°.')
rows=['DOF biologico\tUnita forza/coppia\tMedia\tRMS\tLavoro assoluto[J]']
for dof,r in primary['all_biological_reserves'].items():
    rows.append(f'{dof}\t{r["force_or_torque_unit"]}\t{r["mean"]:+.6f}\t{r["RMS"]:.6f}\t{r["work_J"]["absolute"]:.6f}')
para('\n'.join(rows))
para('Le forzeN e le coppieNm sono tenute separate: non si usa una somma di norme con unità miste. Le reserve di ciascun DOF e il loro lavoro sono conservate nei metrics.json. Le varianti nuove non hanno nessun dato nella finestra primaria, che inizia14,087704688172s: valori primari mancanti non sono zeri.')

para('Detector: calibrazione identica, evidenza distinta per canale e clock')
para('Per400mm la FSM completa3 cicli validi,0 invalidi,0 resync e0 HS annullati; nell’intero episodio sono registrati4HS e4TO. Nella finestra primaria gli onset binari e quelli conservati dall’actor accoppiano4HS/4 e3TO/3, senza candidati extra; rispetto agli eventi nominali daGRF sanaAB07 a20N, bias medioHS−78,463657ms eTO+51,796618ms. L’ultimoHS è osservato e accoppiato, ma il suo margine completo di matching oltre il bordo destro non è osservabile: non è una validazione completa dell’evento di chiusura.')
para('Le tolleranze originali onset sonoHS120ms/TO180ms; confirmedHS50ms/TO80ms; deliveredHS60ms/TO90ms. Con i rispettivi criteri originali, sia confirmed sia delivered accoppiano soltanto1HS/4 e1TO/3. La chiusura di tre cicli FSM non certifica quindi l’accuratezza temporale di tutti i clock. Gli attraversamenti della forza reale sinistra a20N danno6HS/5TO,3HS e3TO accoppiati,3HS e2TO extra e1HS nominale completamente osservato non accoppiato. Il loro bias èHS−3,645355ms eTO+68,295275ms: sono un altro canale, non gli eventi del detector binario.')
para('Per entrambe le varianti nuove tutti i canali primari sono unavailable: l’integrazione termina prima del primoHS nominale dei tre cicli. I contatori FSM rimasti a zero nell’avvio non dimostrano che il detector non funzioni. È un esito dinamico interrotto che impedisce di valutarne le prestazioni. Nessuna soglia, piano, FSM, algoritmo o calibrazione del detector è stata modificata.')

para('Limiti SEA effettivamente registrati')
rows=['Caso\tCampioni\t|u|>=0,999\tClipping effettivo comando\tClamp motore500Nm\tPicco |spring|[Nm]']
for name,label in [('PYLON400_REFERENCE','400mm'),('STANDING_AB07','2regioni'),('STANDING_FOURROI_AB07','4regioni')]:
    s=j['cases'][name]['SEA_all_recorded']['SEA_Knee']
    rows.append(f'{label}\t{s["tau_motor_Nm"]["samples"]}\t{s["normalized_control_saturation"]["samples"]}\t{s["command_limit_evidence"]["unclipped_command_above_unit_bound"]["samples"]}\t{s["motor_torque_clamp"]["samples"]}\t{s["tau_spring_Nm"]["max_abs"]:.6f}')
para('\n'.join(rows))
para('F_opt del ginocchio100Nm limita il riferimento tau_ref=uF_opt; il clamp500Nm riguarda invece l’ingresso del rotore. Nella variante finale118/160 campioni hanno|u|>=0,999 e117/160 sono realmente clippati a−1, con richiesta non limitata massima222,647467Nm. Il clamp del rotore compare solo in2 campioni da1ms. La coppia della molla può superare temporaneamente100Nm. CavigliaF_opt250Nm: nessun comando al limite o clamp del rotore nei tre casi. Questo prova che il ginocchio incontra il limite di comando nell’avvio nuovo, ma non prova da solo la causa del sovraccarico.')

para('Geometria di contatto: dato aggiuntivo e limite causale')
para('La meshSTL e le sfere che generano laGRF sono rappresentazioni diverse. Applicando solamente le trasformazioni statiche salvate alle8 sfere sinistre del profilo di forza invariato, con piano forzaY=−9,50395476mm, si ricavano profondità geometriche positive:400mm2/8,max7,429890mm;2regioni8/8,max46,174436mm;4regioni8/8,max47,380967mm. Le profondità medie sono rispettivamente1,857473/31,216845/37,737863mm. Si tratta di algebra geometrica, senza calcolare forze, effettuare nuove IK/FK o integrare nuovamente.')
para('Questa verifica, aggiunta dopo i test e dichiarata tale, individua una concreta incompatibilità tra l’appoggio imposto alla mesh e la geometria di contatto usata dal banco. Il valore statico47,38mm non è il massimo cammino24,33mm e non è la causa già dimostrata dell’arresto: il reset dinamico è su un fotogramma di cammino, non sulla posa statica. Resta da isolare l’interazione fra montaggio, riferimento IK, contatto virtuale e controllo. Un confronto con una stessa IK può isolare una componente meccanica, ma non sostituisce la preparazione coerente di ogni modello.')

para('Confronto indipendente con Claude e sintesi')
para('Codex ha sigillato le proprie conclusioni complete prima di leggere qualsiasi conclusione visibile di Claude. Claude è stato avviato dal terminale in una chat isolata con modello effettivo claude-fable-5-1, effortmax e nessun tool. Gli sono stati inviati soltanto capsule numeriche e risultati aggregati nel limite esplicitamente autorizzato; non sorgenti, serie temporali grezze, credenziali, accesso ai file o conclusioni locali. La collaborazione è un audit indipendente dei numeri, non una seconda esecuzione OpenSim da parte di Claude.')
para('Concordiamo sulla validità dei vincoli statici dichiarati, sull’arresto precoce delle due varianti nuove, sulla riproducibilità del400mm e sull’impossibilità di giudicare i loro detector sui tre cicli. L’audit iniziale di Claude aveva ipotizzato forza statica nulla dal solo piano sotto la mesh: le profondità delle8 sfere, comunicate dopo come aggregati, smentiscono quell’inferenza. L’audit finale non aveva ricevuto la telemetria dei limiti SEA e la indicava non registrata: l’integrazione numerica successiva fornisce invece i conteggi reali e distingue il limite del comando100Nm dal clamp motore500Nm. I testi visibili sono conservati per verificare il confronto.')
para('Nell’integrazione finale Claude conferma il clipping del comando ginocchio in circa tre quarti dei campioni e il breve clamp separato del rotore. Concordiamo che è un dato registrato, ma non una prova che il limite sia l’origine del problema: può essere una risposta al carico elevato. La sua formulazione «100Nm alla molla» va intesa come limite del riferimento, non come limite rigido della coppia elastica effettiva. Non adottiamo la proposta di aumentareF_opt o di riallineare le sfere senza una controprova causale: tali modifiche non sono state eseguite.')
para('La lunghezza circa434mm deriva dai vincoli geometrici espliciti su AB07 e dal pianoY=0 scelto, non da una scelta a posteriori per migliorare reserve o detector. Il test non dimostra che sia anatomicamente necessaria in assoluto o che questo montaggio sia già dinamicamente qualificato. La variante400mm funziona nel banco esistente ma non rispetta quell’appoggio statico. Il peggioramento introdotto dal nuovo pacchetto di preparazione non può essere imputato automaticamente all’anatomia: stesso soggetto e stessa biologia, ma geometria, mapping protesico, marker e IK coerente cambiano insieme. Il problema residuo verificato è nell’interazione del nuovo montaggio con il banco; il peso causale di ciascuna componente rimane da dimostrare. La calibrazione del detector resta un vincolo fisso.')

para('Verifiche, conservazione e limiti documentali')
para(f'Peer review finalePASS: {peer["protected_source_count"]} sorgenti/artefatti protetti invariati;{peer["reserve_entries_checked"]} combinazioniDOF/finestra controllate per unità, segni, impulso e lavoro;{peer["canonical_PNG_count"]} plot canonici nelle cartelleplot/ del root, tutti ispezionati visivamente. Controllo indipendente della quadraturaRMS nel prefisso comune: errore massimo0N. Fonte numerica principaleanalysis/lean_numeric_summary.json, con metrics/phase originali e receipt di tutti i casi.')
para('Sono conservati i tentativi tecnici precedenti: lettura scalaMesh opzionale corretta, salvataggioJSON parziale ricostruito da blocchi completi e verificato, definizione del guard di tempo reale corretta nella versione2 dell’addendum di sovrapposizione, due stop del parser di inventario prima della scrittura. Non sono retry dinamici né modifica dei risultati. Nessun sorgente di produzione, pluginC++, semanticaSEA, profilo di contatto o detector è stato modificato.')
para('Precisazione dell’audit esterno: una costante riassuntiva del packet finale indicava genericamente50ms per confirmed/delivered; i campi numerici di ciascun evento, inviati nello stesso packet, conservavano i corretti50/80/60/90ms. Questa sintesi usa esclusivamente quei criteri originali per evento, senza introdurre una nuova soglia.')
para('Artefatti principali\nModello finale:full_support_preparation/cases/STANDING_FOURROI_AB07/AB07_SEASEA_marker_calibrated.osim\nIK finale:full_support_preparation/cases/STANDING_FOURROI_AB07/pipeline/02_ik/IK_full_10p360_153p415.mot\nGeometria:geometry/four_roi_support/fit_receipt.json\nRisultati:analysis/lean_numeric_summary.json eanalysis/<case>/metrics.json\nEventi:analysis/phase/<case>/phase_metrics.json\nRun finale:full_support_dynamics/STANDING_FOURROI_AB07/run/receipt.json\nRun400:dynamics/PYLON400_REFERENCE/run/receipt.json\nPlot:../../plot/AB07_standing_geometry_fit_2026_10_07/\nFigura montaggio:../../plot/AB07_standing_geometry_fit_2026_10_07/calibration_geometry_v3.png\nAudit Claude:claude/geometry_audit_visible.txt,claude/final_results_audit_visible.txt,claude/SEA_supplement_audit_visible.txt')

report = AUDIT/'final_synthesis.txt'
readable = []
for part in parts:
    if not part.startswith('Artefatti principali'):
        part = re.sub(r'(?<=[a-zà-ÿ])(?=\d)', ' ', part)
        part = re.sub(r'(?<=\d)(?=[A-Za-zÀ-ÿ])', ' ', part)
        part = re.sub(r'(?<=[a-zà-ÿ])(?=(?:GRF|IK|SEA|FSM|STL|DOF|HS|TO|STO|JSON|CSV|Y=))', ' ', part)
        part = re.sub(r'(?<=\w)(?=[−+])', ' ', part)
        part = part.replace('aumentareF_opt', 'aumentare F_opt').replace('originaliP', 'originali P')
    readable.append(part)
with report.open('x') as f: f.write('\n'.join(readable))
paths = [
    'protocol.json', 'geometry/four_roi_support/fit_receipt.json',
    'geometry/static_mount_comparison.json', 'geometry/static_force_sphere_comparison.json',
    'full_support_preparation/cases/STANDING_FOURROI_AB07/receipt.json',
    'full_support_preparation/cases/STANDING_FOURROI_AB07/AB07_SEASEA_marker_calibrated.osim',
    'full_support_preparation/cases/STANDING_FOURROI_AB07/pipeline/02_ik/IK_full_10p360_153p415.mot',
    'preparation/cases/PYLON400_REFERENCE/receipt.json',
    'preparation/cases/PYLON400_REFERENCE/AB07_SEASEA_marker_calibrated.osim',
    'preparation/cases/PYLON400_REFERENCE/pipeline/02_ik/IK_full_10p360_153p415.mot',
    'full_support_dynamic_protocol.json', 'full_support_execution_closure.json',
    'full_support_dynamics/STANDING_FOURROI_AB07/run/receipt.json',
    'dynamics/PYLON400_REFERENCE/run/receipt.json', 'dynamics/STANDING_AB07/run/receipt.json',
    'analysis/lean_numeric_summary.json', 'root_final_findings_before_claude.json',
    'root_SEA_numerical_addendum_before_claude.json', 'root_common_prefix_check.json',
    'review/final_postprocessing_peer_review.json', 'review/final_standing_scientific_review.json',
    'review/static_force_sphere_peer_review.json', 'review/final_SEA_limit_peer_review.json',
    'claude/scope_receipt.json', 'claude/first_visible_read_receipt.json',
    'claude/final_capsule_scope_receipt_v2.json', 'claude/final_results_completion_and_visible_read_receipt.json',
    'claude/SEA_supplement_scope_receipt.json', 'claude/SEA_supplement_completion_and_visible_read_receipt.json',
    'claude/geometry_audit_visible.txt', 'claude/final_results_audit_visible.txt', 'claude/SEA_supplement_audit_visible.txt',
    'final_synthesis.txt', 'build_final_synthesis.py',
]
for name in j['cases']:
    paths.extend([f'analysis/{name}/metrics.json', f'analysis/phase/{name}/phase_metrics.json'])
manifest = {}
for rel in paths:
    p = AUDIT/rel
    b = p.read_bytes()
    manifest[str(p.relative_to(ROOT))] = {'sha256': hashlib.sha256(b).hexdigest(), 'bytes': len(b)}
figure = ROOT/'plot/AB07_standing_geometry_fit_2026_10_07/calibration_geometry_v3.png'
manifest[str(figure.relative_to(ROOT))] = {'sha256': hashlib.sha256(figure.read_bytes()).hexdigest(), 'bytes': figure.stat().st_size}
receipt = {'created_utc': dt.datetime.now(dt.timezone.utc).isoformat(), 'status': 'COMPLETE',
           'requested_task_completed': True, 'new_geometry_static_gate': 'PASS',
           'new_geometry_dynamic_gate': 'INTERRUPTED', 'new_geometry_primary_detector': 'UNAVAILABLE',
           '400_dynamic_gate': 'COMPLETE', 'detector_recalibrations': 0,
           'new_native_FK_IK_or_integration_for_synthesis': False,
           'canonical_plot_count':21, 'protected_count_at_peer_review':peer['protected_source_count'],
           'claude_model_verified':'claude-fable-5-1', 'claude_effort_requested':'max', 'claude_tools':[],
           'source_manifest':manifest}
with (AUDIT/'final_synthesis_receipt.json').open('x') as f:
    json.dump(receipt, f, indent=2); f.write('\n')
print(json.dumps({'status':receipt['status'], 'report':str(report), 'report_bytes':report.stat().st_size,
                  'bound_artifacts':len(manifest), 'report_sha256':hashlib.sha256(report.read_bytes()).hexdigest()}))
